/*
    Domain communications for Xen Store Daemon.
    Copyright (C) 2005 Rusty Russell IBM Corporation

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; If not, see <http://www.gnu.org/licenses/>.
*/

#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <syslog.h>

#include "utils.h"
#include "talloc.h"
#include "xenstored_core.h"
#include "xenstored_domain.h"
#include "xenstored_transaction.h"
#include "xenstored_watch.h"
#include "xenstored_control.h"

#include <xenevtchn.h>
#include <xenctrl.h>
#include <xen/grant_table.h>

static xc_interface **xc_handle;
xengnttab_handle **xgt_handle;
static evtchn_port_t virq_port;

xenevtchn_handle *xce_handle = NULL;

static struct node_perms dom_release_perms;
static struct node_perms dom_introduce_perms;

struct domain
{
	struct list_head list;

	/* The id of this domain */
	unsigned int domid;

	/* Event channel port */
	evtchn_port_t port;

	/* Domain path in store. */
	char *path;

	/* Shared page. */
	struct xenstore_domain_interface *interface;

	/* The connection associated with this. */
	struct connection *conn;

	/* Generation count at domain introduction time. */
	uint64_t generation;

	/* Have we noticed that this domain is shutdown? */
	bool shutdown;

	/* Has domain been officially introduced? */
	bool introduced;

	/* number of entry from this domain in the store */
	int nbentry;

	/* Amount of memory allocated for this domain. */
	int memory;
	bool soft_quota_reported;
	bool hard_quota_reported;
	time_t mem_last_msg;
#define MEM_WARN_MINTIME_SEC 10

	/* number of watch for this domain */
	int nbwatch;

	/* Number of outstanding requests. */
	int nboutstanding;

	/* write rate limit */
	wrl_creditt wrl_credit; /* [ -wrl_config_writecost, +_dburst ] */
	struct wrl_timestampt wrl_timestamp;
	bool wrl_delay_logged;
};

static LIST_HEAD(domains);

static bool check_indexes(XENSTORE_RING_IDX cons, XENSTORE_RING_IDX prod)
{
	return ((prod - cons) <= XENSTORE_RING_SIZE);
}

static void *get_output_chunk(XENSTORE_RING_IDX cons,
			      XENSTORE_RING_IDX prod,
			      char *buf, uint32_t *len)
{
	*len = XENSTORE_RING_SIZE - MASK_XENSTORE_IDX(prod);
	if ((XENSTORE_RING_SIZE - (prod - cons)) < *len)
		*len = XENSTORE_RING_SIZE - (prod - cons);
	return buf + MASK_XENSTORE_IDX(prod);
}

static const void *get_input_chunk(XENSTORE_RING_IDX cons,
				   XENSTORE_RING_IDX prod,
				   const char *buf, uint32_t *len)
{
	*len = XENSTORE_RING_SIZE - MASK_XENSTORE_IDX(cons);
	if ((prod - cons) < *len)
		*len = prod - cons;
	return buf + MASK_XENSTORE_IDX(cons);
}

static int writechn(struct connection *conn,
		    const void *data, unsigned int len)
{
	uint32_t avail;
	void *dest;
	struct xenstore_domain_interface *intf = conn->domain->interface;
	XENSTORE_RING_IDX cons, prod;

	/* Must read indexes once, and before anything else, and verified. */
	cons = intf->rsp_cons;
	prod = intf->rsp_prod;
	xen_mb();

	if (!check_indexes(cons, prod)) {
		errno = EIO;
		return -1;
	}

	dest = get_output_chunk(cons, prod, intf->rsp, &avail);
	if (avail < len)
		len = avail;

	memcpy(dest, data, len);
	xen_mb();
	intf->rsp_prod += len;

	xenevtchn_notify(xce_handle, conn->domain->port);

	return len;
}

static int readchn(struct connection *conn, void *data, unsigned int len)
{
	uint32_t avail;
	const void *src;
	struct xenstore_domain_interface *intf = conn->domain->interface;
	XENSTORE_RING_IDX cons, prod;

	/* Must read indexes once, and before anything else, and verified. */
	cons = intf->req_cons;
	prod = intf->req_prod;
	xen_mb();

	if (!check_indexes(cons, prod)) {
		errno = EIO;
		return -1;
	}

	src = get_input_chunk(cons, prod, intf->req, &avail);
	if (avail < len)
		len = avail;

	memcpy(data, src, len);
	xen_mb();
	intf->req_cons += len;

	xenevtchn_notify(xce_handle, conn->domain->port);

	return len;
}

static bool domain_can_write(struct connection *conn)
{
	struct xenstore_domain_interface *intf = conn->domain->interface;

	return ((intf->rsp_prod - intf->rsp_cons) != XENSTORE_RING_SIZE);
}

static bool domain_can_read(struct connection *conn)
{
	struct xenstore_domain_interface *intf = conn->domain->interface;

	if (domain_is_unprivileged(conn)) {
		if (conn->domain->wrl_credit < 0)
			return false;
		if (conn->domain->nboutstanding >= quota_req_outstanding)
			return false;
		if (conn->domain->memory >= quota_memory_per_domain_hard &&
		    quota_memory_per_domain_hard)
			return false;
	}

	return (intf->req_cons != intf->req_prod);
}

static const struct interface_funcs domain_funcs = {
	.write = writechn,
	.read = readchn,
	.can_write = domain_can_write,
	.can_read = domain_can_read,
};

static void *map_interface(domid_t domid)
{
	return xengnttab_map_grant_ref(*xgt_handle, domid,
				       GNTTAB_RESERVED_XENSTORE,
				       PROT_READ|PROT_WRITE);
}

static void unmap_interface(void *interface)
{
	xengnttab_unmap(*xgt_handle, interface, 1);
}

static void remove_domid_from_perm(struct node_perms *perms,
				   struct domain *domain)
{
	unsigned int cur, new;

	if (perms->p[0].id == domain->domid)
		perms->p[0].id = priv_domid;

	for (cur = new = 1; cur < perms->num; cur++) {
		if (perms->p[cur].id == domain->domid)
			continue;

		if (new != cur)
			perms->p[new] = perms->p[cur];

		new++;
	}

	perms->num = new;
}

static int domain_tree_remove_sub(const void *ctx, struct connection *conn,
				  struct node *node, void *arg)
{
	struct domain *domain = arg;
	TDB_DATA key;
	int ret = WALK_TREE_OK;

	if (node->perms.p[0].id != domain->domid)
		return WALK_TREE_OK;

	if (keep_orphans) {
		set_tdb_key(node->name, &key);
		domain->nbentry--;
		node->perms.p[0].id = priv_domid;
		node->acc.memory = 0;
		domain_entry_inc(NULL, node);
		if (write_node_raw(NULL, &key, node, true)) {
			/* That's unfortunate. We only can try to continue. */
			syslog(LOG_ERR,
			       "error when moving orphaned node %s to dom0\n",
			       node->name);
		} else
			trace("orphaned node %s moved to dom0\n", node->name);
	} else {
		if (rm_node(NULL, ctx, node->name)) {
			/* That's unfortunate. We only can try to continue. */
			syslog(LOG_ERR,
			       "error when deleting orphaned node %s\n",
			       node->name);
		} else
			trace("orphaned node %s deleted\n", node->name);

		/* Skip children in all cases in order to avoid more errors. */
		ret = WALK_TREE_SKIP_CHILDREN;
	}

	return domain->nbentry > 0 ? ret : WALK_TREE_SUCCESS_STOP;
}

static void domain_tree_remove(struct domain *domain)
{
	int ret;
	struct walk_funcs walkfuncs = { .enter = domain_tree_remove_sub };

	if (domain->nbentry > 0) {
		ret = walk_node_tree(domain, NULL, "/", &walkfuncs, domain);
		if (ret == WALK_TREE_ERROR_STOP)
			syslog(LOG_ERR,
			       "error when looking for orphaned nodes\n");
	}

	remove_domid_from_perm(&dom_release_perms, domain);
	remove_domid_from_perm(&dom_introduce_perms, domain);
}

static int destroy_domain(void *_domain)
{
	struct domain *domain = _domain;

	domain_tree_remove(domain);

	list_del(&domain->list);

	if (!domain->introduced)
		return 0;

	if (domain->port) {
		if (xenevtchn_unbind(xce_handle, domain->port) == -1)
			eprintf("> Unbinding port %i failed!\n", domain->port);
	}

	if (domain->interface) {
		/* Domain 0 was mapped by dom0_init, so it must be unmapped
		   using munmap() and not the grant unmap call. */
		if (domain->domid == 0)
			unmap_xenbus(domain->interface);
		else
			unmap_interface(domain->interface);
	}

	fire_watches(NULL, domain, "@releaseDomain", NULL, true, NULL);

	wrl_domain_destroy(domain);

	return 0;
}

static bool get_domain_info(unsigned int domid, xc_domaininfo_t *dominfo)
{
	return xc_domain_getinfo_single(*xc_handle, domid, dominfo) == 0;
}

void check_domains(void)
{
	xc_domaininfo_t dominfo;
	struct domain *domain;
	struct connection *conn;
	int notify = 0;
	bool dom_valid;

 again:
	list_for_each_entry(domain, &domains, list) {
		dom_valid = get_domain_info(domain->domid, &dominfo);
		if (!domain->introduced) {
			if (!dom_valid) {
				talloc_free(domain);
				goto again;
			}
			continue;
		}
		if (dom_valid) {
			if ((dominfo.flags & XEN_DOMINF_shutdown)
			    && !domain->shutdown) {
				domain->shutdown = true;
				notify = 1;
			}
			if (!(dominfo.flags & XEN_DOMINF_dying))
				continue;
		}
		if (domain->conn) {
			/* domain is a talloc child of domain->conn. */
			conn = domain->conn;
			domain->conn = NULL;
			talloc_unlink(talloc_autofree_context(), conn);
			notify = 0; /* destroy_domain() fires the watch */
			goto again;
		}
	}

	if (notify)
		fire_watches(NULL, NULL, "@releaseDomain", NULL, true, NULL);
}

/* We scan all domains rather than use the information given here. */
void handle_event(void)
{
	evtchn_port_t port;

	if ((port = xenevtchn_pending(xce_handle)) == -1)
		barf_perror("Failed to read from event fd");

	if (port == virq_port)
		check_domains();

	if (xenevtchn_unmask(xce_handle, port) == -1)
		barf_perror("Failed to write to event fd");
}

static bool domid_is_unprivileged(unsigned int domid)
{
	return domid != 0 && domid != priv_domid;
}

bool domain_is_unprivileged(struct connection *conn)
{
	return conn && conn->domain &&
	       domid_is_unprivileged(conn->domain->domid);
}

static char *talloc_domain_path(const void *context, unsigned int domid)
{
	return talloc_asprintf(context, "/local/domain/%u", domid);
}

static struct domain *find_domain_struct(unsigned int domid)
{
	struct domain *i;

	list_for_each_entry(i, &domains, list) {
		if (i->domid == domid)
			return i;
	}
	return NULL;
}

int domain_get_quota(const void *ctx, struct connection *conn,
		     unsigned int domid)
{
	struct domain *d = find_domain_struct(domid);
	char *resp;
	int ta;

	if (!d)
		return ENOENT;

	ta = d->conn ? d->conn->transaction_started : 0;
	resp = talloc_asprintf(ctx, "Domain %u:\n", domid);
	if (!resp)
		return ENOMEM;

#define ent(t, e) \
	resp = talloc_asprintf_append(resp, "%-16s: %8d\n", #t, e); \
	if (!resp) return ENOMEM

	ent(nodes, d->nbentry);
	ent(watches, d->nbwatch);
	ent(transactions, ta);
	ent(outstanding, d->nboutstanding);
	ent(memory, d->memory);

#undef ent

	send_reply(conn, XS_CONTROL, resp, strlen(resp) + 1);

	return 0;
}

static struct domain *alloc_domain(const void *context, unsigned int domid)
{
	struct domain *domain;

	domain = talloc_zero(context, struct domain);
	if (!domain) {
		errno = ENOMEM;
		return NULL;
	}

	domain->domid = domid;
	domain->generation = generation;
	domain->introduced = false;

	talloc_set_destructor(domain, destroy_domain);

	list_add(&domain->list, &domains);

	return domain;
}

static struct domain *find_or_alloc_domain(const void *ctx, unsigned int domid)
{
	struct domain *domain;

	domain = find_domain_struct(domid);
	return domain ? : alloc_domain(ctx, domid);
}

static struct domain *find_or_alloc_existing_domain(unsigned int domid)
{
	struct domain *domain;
	xc_domaininfo_t dominfo;

	domain = find_domain_struct(domid);
	if (!domain && get_domain_info(domid, &dominfo))
		domain = alloc_domain(NULL, domid);

	return domain;
}

static int new_domain(struct domain *domain, int port, bool restore)
{
	int rc;

	domain->port = 0;
	domain->shutdown = false;
	domain->path = talloc_domain_path(domain, domain->domid);
	if (!domain->path) {
		errno = ENOMEM;
		return errno;
	}

	wrl_domain_new(domain);

	if (restore)
		domain->port = port;
	else {
		/* Tell kernel we're interested in this event. */
		rc = xenevtchn_bind_interdomain(xce_handle, domain->domid,
						port);
		if (rc == -1)
			return errno;
		domain->port = rc;
	}

	domain->introduced = true;

	domain->conn = new_connection(&domain_funcs);
	if (!domain->conn)  {
		errno = ENOMEM;
		return errno;
	}

	domain->conn->domain = domain;
	domain->conn->id = domain->domid;

	return 0;
}


static struct domain *find_domain_by_domid(unsigned int domid)
{
	struct domain *d;

	d = find_domain_struct(domid);

	return (d && d->introduced) ? d : NULL;
}

static void domain_conn_reset(struct domain *domain)
{
	struct connection *conn = domain->conn;

	conn_delete_all_watches(conn);
	conn_delete_all_transactions(conn);
	conn_free_buffered_data(conn);

	talloc_free(conn->in);

	domain->interface->req_cons = domain->interface->req_prod = 0;
	domain->interface->rsp_cons = domain->interface->rsp_prod = 0;
	xen_wmb();
}

/*
 * Keep the connection alive but stop processing any new request or sending
 * reponse. This is to allow sending @releaseDomain watch event at the correct
 * moment and/or to allow the connection to restart (not yet implemented).
 *
 * All watches, transactions, buffers will be freed.
 */
void ignore_connection(struct connection *conn, unsigned int err)
{
	trace("CONN %p ignored, reason %u\n", conn, err);

	if (conn->domain && conn->domain->interface)
		conn->domain->interface->error = err;

	conn->is_ignored = true;
	conn_delete_all_watches(conn);
	conn_delete_all_transactions(conn);
	conn_free_buffered_data(conn);

	talloc_free(conn->in);
	conn->in = NULL;
	/* if this is a socket connection, drop it now */
	if (conn->fd >= 0)
		talloc_free(conn);
}

static struct domain *introduce_domain(const void *ctx,
				       unsigned int domid,
				       evtchn_port_t port, bool restore)
{
	struct domain *domain;
	int rc;
	struct xenstore_domain_interface *interface;
	bool is_master_domain = (domid == xenbus_master_domid());

	domain = find_or_alloc_domain(ctx, domid);
	if (!domain)
		return NULL;

	if (!domain->introduced) {
		interface = is_master_domain ? xenbus_map()
					     : map_interface(domid);
		if (!interface && !restore)
			return NULL;
		if (new_domain(domain, port, restore)) {
			rc = errno;
			if (interface) {
				if (is_master_domain)
					unmap_xenbus(interface);
				else
					unmap_interface(interface);
			}
			errno = rc;
			return NULL;
		}
		domain->interface = interface;

		if (is_master_domain)
			setup_structure(restore);

		/* Now domain belongs to its connection. */
		talloc_steal(domain->conn, domain);

		if (!is_master_domain && !restore)
			fire_watches(NULL, ctx, "@introduceDomain", NULL,
				     true, NULL);
	} else {
		/* Use XS_INTRODUCE for recreating the xenbus event-channel. */
		if (domain->port)
			xenevtchn_unbind(xce_handle, domain->port);
		rc = xenevtchn_bind_interdomain(xce_handle, domid, port);
		domain->port = (rc == -1) ? 0 : rc;
	}

	return domain;
}

/* domid, gfn, evtchn, path */
int do_introduce(const void *ctx, struct connection *conn,
		 struct buffered_data *in)
{
	struct domain *domain;
	char *vec[3];
	unsigned int domid;
	evtchn_port_t port;

	if (get_strings(in, vec, ARRAY_SIZE(vec)) < ARRAY_SIZE(vec))
		return EINVAL;

	domid = atoi(vec[0]);
	/* Ignore the gfn, we don't need it. */
	port = atoi(vec[2]);

	/* Sanity check args. */
	if (port <= 0)
		return EINVAL;

	domain = introduce_domain(ctx, domid, port, false);
	if (!domain)
		return errno;

	domain_conn_reset(domain);

	if (domain->interface != NULL &&
	    domain->interface->connection == XENSTORE_RECONNECT) {
		/* Notify the domain that xenstore is available */
		domain->interface->connection = XENSTORE_CONNECTED;
		xenevtchn_notify(xce_handle, domain->port);
	}

	send_ack(conn, XS_INTRODUCE);

	return 0;
}

static struct domain *find_connected_domain(unsigned int domid)
{
	struct domain *domain;

	domain = find_domain_by_domid(domid);
	if (!domain)
		return ERR_PTR(-ENOENT);
	if (!domain->conn)
		return ERR_PTR(-EINVAL);
	return domain;
}

int do_set_target(const void *ctx, struct connection *conn,
		  struct buffered_data *in)
{
	char *vec[2];
	unsigned int domid, tdomid;
        struct domain *domain, *tdomain;
	if (get_strings(in, vec, ARRAY_SIZE(vec)) < ARRAY_SIZE(vec))
		return EINVAL;

	domid = atoi(vec[0]);
	tdomid = atoi(vec[1]);

        domain = find_connected_domain(domid);
	if (IS_ERR(domain))
		return -PTR_ERR(domain);

        tdomain = find_connected_domain(tdomid);
	if (IS_ERR(tdomain))
		return -PTR_ERR(tdomain);

        talloc_reference(domain->conn, tdomain->conn);
        domain->conn->target = tdomain->conn;

	send_ack(conn, XS_SET_TARGET);

	return 0;
}

static struct domain *onearg_domain(struct connection *conn,
				    struct buffered_data *in)
{
	const char *domid_str = onearg(in);
	unsigned int domid;

	if (!domid_str)
		return ERR_PTR(-EINVAL);

	domid = atoi(domid_str);
	if (!domid)
		return ERR_PTR(-EINVAL);

	return find_connected_domain(domid);
}

/* domid */
int do_release(const void *ctx, struct connection *conn,
	       struct buffered_data *in)
{
	struct domain *domain;

	domain = onearg_domain(conn, in);
	if (IS_ERR(domain))
		return -PTR_ERR(domain);

	talloc_free(domain->conn);

	send_ack(conn, XS_RELEASE);

	return 0;
}

int do_resume(const void *ctx, struct connection *conn,
	      struct buffered_data *in)
{
	struct domain *domain;

	domain = onearg_domain(conn, in);
	if (IS_ERR(domain))
		return -PTR_ERR(domain);

	domain->shutdown = false;

	send_ack(conn, XS_RESUME);

	return 0;
}

int do_get_domain_path(const void *ctx, struct connection *conn,
		       struct buffered_data *in)
{
	char *path;
	const char *domid_str = onearg(in);

	if (!domid_str)
		return EINVAL;

	path = talloc_domain_path(ctx, atoi(domid_str));
	if (!path)
		return errno;

	send_reply(conn, XS_GET_DOMAIN_PATH, path, strlen(path) + 1);

	return 0;
}

int do_is_domain_introduced(const void *ctx, struct connection *conn,
			    struct buffered_data *in)
{
	int result;
	unsigned int domid;
	const char *domid_str = onearg(in);

	if (!domid_str)
		return EINVAL;

	domid = atoi(domid_str);
	if (domid == DOMID_SELF)
		result = 1;
	else
		result = (find_domain_by_domid(domid) != NULL);

	send_reply(conn, XS_IS_DOMAIN_INTRODUCED, result ? "T" : "F", 2);

	return 0;
}

/* Allow guest to reset all watches */
int do_reset_watches(const void *ctx, struct connection *conn,
		     struct buffered_data *in)
{
	conn_delete_all_watches(conn);
	conn_delete_all_transactions(conn);

	send_ack(conn, XS_RESET_WATCHES);

	return 0;
}

static int close_xc_handle(void *_handle)
{
	xc_interface_close(*(xc_interface**)_handle);
	return 0;
}

static int close_xgt_handle(void *_handle)
{
	xengnttab_close(*(xengnttab_handle **)_handle);
	return 0;
}

/* Returns the implicit path of a connection (only domains have this) */
const char *get_implicit_path(const struct connection *conn)
{
	if (!conn->domain)
		return "/local/domain/0";
	return conn->domain->path;
}

static int set_dom_perms_default(struct node_perms *perms)
{
	perms->num = 1;
	perms->p = talloc_array(NULL, struct xs_permissions, perms->num);
	if (!perms->p)
		return -1;
	perms->p->id = 0;
	perms->p->perms = XS_PERM_NONE;

	return 0;
}

static struct node_perms *get_perms_special(const char *name)
{
	if (!strcmp(name, "@releaseDomain"))
		return &dom_release_perms;
	if (!strcmp(name, "@introduceDomain"))
		return &dom_introduce_perms;
	return NULL;
}

int set_perms_special(struct connection *conn, const char *name,
		      struct node_perms *perms)
{
	struct node_perms *p;

	p = get_perms_special(name);
	if (!p)
		return EINVAL;

	if ((perm_for_conn(conn, p) & (XS_PERM_WRITE | XS_PERM_OWNER)) !=
	    (XS_PERM_WRITE | XS_PERM_OWNER))
		return EACCES;

	p->num = perms->num;
	talloc_free(p->p);
	p->p = perms->p;
	talloc_steal(NULL, perms->p);

	return 0;
}

bool check_perms_special(const char *name, struct connection *conn)
{
	struct node_perms *p;

	p = get_perms_special(name);
	if (!p)
		return false;

	return perm_for_conn(conn, p) & XS_PERM_READ;
}

void dom0_init(void)
{
	evtchn_port_t port;
	struct domain *dom0;

	port = xenbus_evtchn();
	if (port == -1)
		barf_perror("Failed to initialize dom0 port");

	dom0 = introduce_domain(NULL, xenbus_master_domid(), port, false);
	if (!dom0)
		barf_perror("Failed to initialize dom0");

	xenevtchn_notify(xce_handle, dom0->port);
}

void domain_init(int evtfd)
{
	int rc;

	xc_handle = talloc(talloc_autofree_context(), xc_interface*);
	if (!xc_handle)
		barf_perror("Failed to allocate domain handle");

	*xc_handle = xc_interface_open(0,0,0);
	if (!*xc_handle)
		barf_perror("Failed to open connection to hypervisor");

	talloc_set_destructor(xc_handle, close_xc_handle);

	xgt_handle = talloc(talloc_autofree_context(), xengnttab_handle*);
	if (!xgt_handle)
		barf_perror("Failed to allocate domain gnttab handle");

	*xgt_handle = xengnttab_open(NULL, 0);
	if (*xgt_handle == NULL)
		barf_perror("Failed to open connection to gnttab");

	/*
	 * Allow max number of domains for mappings. We allow one grant per
	 * domain so the theoretical maximum is DOMID_FIRST_RESERVED.
	 */
	xengnttab_set_max_grants(*xgt_handle, DOMID_FIRST_RESERVED);

	talloc_set_destructor(xgt_handle, close_xgt_handle);

	if (evtfd < 0)
		xce_handle = xenevtchn_open(NULL, XENEVTCHN_NO_CLOEXEC);
	else
		xce_handle = xenevtchn_fdopen(NULL, evtfd, 0);

	if (xce_handle == NULL)
		barf_perror("Failed to open evtchn device");

	if (set_dom_perms_default(&dom_release_perms) ||
	    set_dom_perms_default(&dom_introduce_perms))
		barf_perror("Failed to set special permissions");

	if ((rc = xenevtchn_bind_virq(xce_handle, VIRQ_DOM_EXC)) == -1)
		barf_perror("Failed to bind to domain exception virq port");
	virq_port = rc;
}

void domain_deinit(void)
{
	if (virq_port)
		xenevtchn_unbind(xce_handle, virq_port);
}

int domain_entry_inc(struct connection *conn, struct node *node)
{
	struct domain *d;
	unsigned int domid;

	if (!node->perms.p)
		return 0;

	domid = node->perms.p[0].id;

	if (conn && conn->transaction) {
		transaction_entry_inc(conn->transaction, domid);
	} else {
		d = (conn && domid == conn->id && conn->domain) ? conn->domain
		    : find_or_alloc_existing_domain(domid);
		if (d)
			d->nbentry++;
		else
			return ENOMEM;
	}

	return 0;
}

/*
 * Check whether a domain was created before or after a specific generation
 * count (used for testing whether a node permission is older than a domain).
 *
 * Return values:
 *  0: domain has higher generation count (it is younger than a node with the
 *     given count), or domain isn't existing any longer
 *  1: domain is older than the node
 */
static int chk_domain_generation(unsigned int domid, uint64_t gen)
{
	struct domain *d;

	if (!xc_handle && domid == 0)
		return 1;

	d = find_domain_struct(domid);

	return (d && d->generation <= gen) ? 1 : 0;
}

/*
 * Allocate all missing struct domain referenced by a permission set.
 * Any permission entries for not existing domains will be marked to be
 * ignored.
 */
int domain_alloc_permrefs(struct node_perms *perms)
{
	unsigned int i, domid;
	struct domain *d;
	xc_domaininfo_t dominfo;

	for (i = 0; i < perms->num; i++) {
		domid = perms->p[i].id;
		d = find_domain_struct(domid);
		if (!d) {
			if (!get_domain_info(domid, &dominfo))
				perms->p[i].perms |= XS_PERM_IGNORE;
			else if (!alloc_domain(NULL, domid))
				return ENOMEM;
		}
	}

	return 0;
}

/*
 * Remove permissions for no longer existing domains in order to avoid a new
 * domain with the same domid inheriting the permissions.
 */
int domain_adjust_node_perms(struct node *node)
{
	unsigned int i;
	int ret;

	for (i = 1; i < node->perms.num; i++) {
		if (node->perms.p[i].perms & XS_PERM_IGNORE)
			continue;
		ret = chk_domain_generation(node->perms.p[i].id,
					    node->generation);
		if (!ret)
			node->perms.p[i].perms |= XS_PERM_IGNORE;
	}

	return 0;
}

void domain_entry_dec(struct connection *conn, struct node *node)
{
	struct domain *d;
	unsigned int domid;

	if (!node->perms.p)
		return;

	domid = node->perms.p ? node->perms.p[0].id : conn->id;

	if (conn && conn->transaction) {
		transaction_entry_dec(conn->transaction, domid);
	} else {
		d = (conn && domid == conn->id && conn->domain) ? conn->domain
		    : find_domain_struct(domid);
		if (d) {
			d->nbentry--;
		} else {
			errno = ENOENT;
			corrupt(conn,
				"Node \"%s\" owned by non-existing domain %u\n",
				node->name, domid);
		}
	}
}

int domain_entry_fix(unsigned int domid, int num, bool update)
{
	struct domain *d;
	int cnt;

	if (update) {
		d = find_domain_struct(domid);
		assert(d);
	} else {
		/*
		 * We are called first with update == false in order to catch
		 * any error. So do a possible allocation and check for error
		 * only in this case, as in the case of update == true nothing
		 * can go wrong anymore as the allocation already happened.
		 */
		d = find_or_alloc_existing_domain(domid);
		if (!d)
			return -1;
	}

	cnt = d->nbentry + num;

	if (update) {
		assert(cnt >= 0);
		d->nbentry = cnt;
	} else if (cnt < 0) {
		/*
		 * In a transaction when a node is being added/removed AND
		 * the same node has been added/removed outside the
		 * transaction in parallel, the result value may be negative.
		 * This is no problem, as the transaction will fail due to
		 * the resulting conflict. So override 'cnt'.
		 */
		cnt = 0;
	}

	return domid_is_unprivileged(domid) ? cnt : 0;
}

int domain_entry(struct connection *conn)
{
	return (domain_is_unprivileged(conn))
		? conn->domain->nbentry
		: 0;
}

static bool domain_chk_quota(struct domain *domain, int mem)
{
	time_t now;

	if (!domain || !domid_is_unprivileged(domain->domid) ||
	    (domain->conn && domain->conn->is_ignored))
		return false;

	now = time(NULL);

	if (mem >= quota_memory_per_domain_hard &&
	    quota_memory_per_domain_hard) {
		if (domain->hard_quota_reported)
			return true;
		syslog(LOG_ERR, "Domain %u exceeds hard memory quota, Xenstore interface to domain stalled\n",
		       domain->domid);
		domain->mem_last_msg = now;
		domain->hard_quota_reported = true;
		return true;
	}

	if (now - domain->mem_last_msg >= MEM_WARN_MINTIME_SEC) {
		if (domain->hard_quota_reported) {
			domain->mem_last_msg = now;
			domain->hard_quota_reported = false;
			syslog(LOG_INFO, "Domain %u below hard memory quota again\n",
			       domain->domid);
		}
		if (mem >= quota_memory_per_domain_soft &&
		    quota_memory_per_domain_soft &&
		    !domain->soft_quota_reported) {
			domain->mem_last_msg = now;
			domain->soft_quota_reported = true;
			syslog(LOG_WARNING, "Domain %u exceeds soft memory quota\n",
			       domain->domid);
		}
		if (mem < quota_memory_per_domain_soft &&
		    domain->soft_quota_reported) {
			domain->mem_last_msg = now;
			domain->soft_quota_reported = false;
			syslog(LOG_INFO, "Domain %u below soft memory quota again\n",
			       domain->domid);
		}

	}

	return false;
}

int domain_memory_add(unsigned int domid, int mem, bool no_quota_check)
{
	struct domain *domain;

	domain = find_domain_struct(domid);
	if (domain) {
		/*
		 * domain_chk_quota() will print warning and also store whether
		 * the soft/hard quota has been hit. So check no_quota_check
		 * *after*.
		 */
		if (domain_chk_quota(domain, domain->memory + mem) &&
		    !no_quota_check)
			return ENOMEM;
		domain->memory += mem;
	} else {
		/*
		 * The domain the memory is to be accounted for should always
		 * exist, as accounting is done either for a domain related to
		 * the current connection, or for the domain owning a node
		 * (which is always existing, as the owner of the node is
		 * tested to exist and deleted or replaced by domid 0 if not).
		 * So not finding the related domain MUST be an error in the
		 * data base.
		 */
		errno = ENOENT;
		corrupt(NULL, "Accounting called for non-existing domain %u\n",
			domid);
		return ENOENT;
	}

	return 0;
}

void domain_watch_inc(struct connection *conn)
{
	if (!conn || !conn->domain)
		return;
	conn->domain->nbwatch++;
}

void domain_watch_dec(struct connection *conn)
{
	if (!conn || !conn->domain)
		return;
	if (conn->domain->nbwatch)
		conn->domain->nbwatch--;
}

int domain_watch(struct connection *conn)
{
	return (domain_is_unprivileged(conn))
		? conn->domain->nbwatch
		: 0;
}

void domain_outstanding_inc(struct connection *conn)
{
	if (!conn || !conn->domain)
		return;
	conn->domain->nboutstanding++;
}

void domain_outstanding_dec(struct connection *conn)
{
	if (!conn || !conn->domain)
		return;
	conn->domain->nboutstanding--;
}

void domain_outstanding_domid_dec(unsigned int domid)
{
	struct domain *d = find_domain_by_domid(domid);

	if (d)
		d->nboutstanding--;
}

static wrl_creditt wrl_config_writecost      = WRL_FACTOR;
static wrl_creditt wrl_config_rate           = WRL_RATE   * WRL_FACTOR;
static wrl_creditt wrl_config_dburst         = WRL_DBURST * WRL_FACTOR;
static wrl_creditt wrl_config_gburst         = WRL_GBURST * WRL_FACTOR;
static wrl_creditt wrl_config_newdoms_dburst =
	                         WRL_DBURST * WRL_NEWDOMS * WRL_FACTOR;

long wrl_ntransactions;

static long wrl_ndomains;
static wrl_creditt wrl_reserve; /* [-wrl_config_newdoms_dburst, +_gburst ] */
static time_t wrl_log_last_warning; /* 0: no previous warning */

void wrl_gettime_now(struct wrl_timestampt *now_wt)
{
	struct timespec now_ts;
	int r;

	r = clock_gettime(CLOCK_MONOTONIC, &now_ts);
	if (r)
		barf_perror("Could not find time (clock_gettime failed)");

	now_wt->sec = now_ts.tv_sec;
	now_wt->msec = now_ts.tv_nsec / 1000000;
}

static void wrl_xfer_credit(wrl_creditt *debit,  wrl_creditt debit_floor,
			    wrl_creditt *credit, wrl_creditt credit_ceil)
	/*
	 * Transfers zero or more credit from "debit" to "credit".
	 * Transfers as much as possible while maintaining
	 * debit >= debit_floor and credit <= credit_ceil.
	 * (If that's violated already, does nothing.)
	 *
	 * Sufficient conditions to avoid overflow, either of:
	 *  |every argument| <= 0x3fffffff
	 *  |every argument| <= 1E9
	 *  |every argument| <= WRL_CREDIT_MAX
	 * (And this condition is preserved.)
	 */
{
	wrl_creditt xfer = MIN( *debit      - debit_floor,
			        credit_ceil - *credit      );
	if (xfer > 0) {
		*debit -= xfer;
		*credit += xfer;
	}
}

void wrl_domain_new(struct domain *domain)
{
	domain->wrl_credit = 0;
	wrl_gettime_now(&domain->wrl_timestamp);
	wrl_ndomains++;
	/* Steal up to DBURST from the reserve */
	wrl_xfer_credit(&wrl_reserve, -wrl_config_newdoms_dburst,
			&domain->wrl_credit, wrl_config_dburst);
}

void wrl_domain_destroy(struct domain *domain)
{
	wrl_ndomains--;
	/*
	 * Don't bother recalculating domain's credit - this just
	 * means we don't give the reserve the ending domain's credit
	 * for time elapsed since last update.
	 */
	wrl_xfer_credit(&domain->wrl_credit, 0,
			&wrl_reserve, wrl_config_dburst);
}

void wrl_credit_update(struct domain *domain, struct wrl_timestampt now)
{
	/*
	 * We want to calculate
	 *    credit += (now - timestamp) * RATE / ndoms;
	 * But we want it to saturate, and to avoid floating point.
	 * To avoid rounding errors from constantly adding small
	 * amounts of credit, we only add credit for whole milliseconds.
	 */
	long seconds      = now.sec -  domain->wrl_timestamp.sec;
	long milliseconds = now.msec - domain->wrl_timestamp.msec;
	long msec;
	int64_t denom, num;
	wrl_creditt surplus;

	seconds = MIN(seconds, 1000*1000); /* arbitrary, prevents overflow */
	msec = seconds * 1000 + milliseconds;

	if (msec < 0)
                /* shouldn't happen with CLOCK_MONOTONIC */
		msec = 0;

	/* 32x32 -> 64 cannot overflow */
	denom = (int64_t)msec * wrl_config_rate;
	num  =  (int64_t)wrl_ndomains * 1000;
	/* denom / num <= 1E6 * wrl_config_rate, so with
	   reasonable wrl_config_rate, denom / num << 2^64 */

	/* at last! */
	domain->wrl_credit = MIN( (int64_t)domain->wrl_credit + denom / num,
				  WRL_CREDIT_MAX );
	/* (maybe briefly violating the DBURST cap on wrl_credit) */

	/* maybe take from the reserve to make us nonnegative */
	wrl_xfer_credit(&wrl_reserve,        0,
			&domain->wrl_credit, 0);

	/* return any surplus (over DBURST) to the reserve */
	surplus = 0;
	wrl_xfer_credit(&domain->wrl_credit, wrl_config_dburst,
			&surplus,            WRL_CREDIT_MAX);
	wrl_xfer_credit(&surplus,     0,
			&wrl_reserve, wrl_config_gburst);
	/* surplus is now implicitly discarded */

	domain->wrl_timestamp = now;

	trace("wrl: dom %4d %6ld  msec  %9ld credit   %9ld reserve"
	      "  %9ld discard\n",
	      domain->domid,
	      msec,
	      (long)domain->wrl_credit, (long)wrl_reserve,
	      (long)surplus);
}

void wrl_check_timeout(struct domain *domain,
		       struct wrl_timestampt now,
		       int *ptimeout)
{
	uint64_t num, denom;
	int wakeup;

	wrl_credit_update(domain, now);

	if (domain->wrl_credit >= 0)
		/* not blocked */
		return;

	if (!*ptimeout)
		/* already decided on immediate wakeup,
		   so no need to calculate our timeout */
		return;

	/* calculate  wakeup = now + -credit / (RATE / ndoms); */

	/* credit cannot go more -ve than one transaction,
	 * so the first multiplication cannot overflow even 32-bit */
	num   = (uint64_t)(-domain->wrl_credit * 1000) * wrl_ndomains;
	denom = wrl_config_rate;

	wakeup = MIN( num / denom /* uint64_t */, INT_MAX );
	if (*ptimeout==-1 || wakeup < *ptimeout)
		*ptimeout = wakeup;

	trace("wrl: domain %u credit=%ld (reserve=%ld) SLEEPING for %d\n",
	      domain->domid,
	      (long)domain->wrl_credit, (long)wrl_reserve,
	      wakeup);
}

#define WRL_LOG(now, ...) \
	(syslog(LOG_WARNING, "write rate limit: " __VA_ARGS__))

void wrl_apply_debit_actual(struct domain *domain)
{
	struct wrl_timestampt now;

	if (!domain || !domid_is_unprivileged(domain->domid))
		/* sockets and privileged domain escape the write rate limit */
		return;

	wrl_gettime_now(&now);
	wrl_credit_update(domain, now);

	domain->wrl_credit -= wrl_config_writecost;
	trace("wrl: domain %u credit=%ld (reserve=%ld)\n",
	      domain->domid,
	      (long)domain->wrl_credit, (long)wrl_reserve);

	if (domain->wrl_credit < 0) {
		if (!domain->wrl_delay_logged) {
			domain->wrl_delay_logged = true;
			WRL_LOG(now, "domain %ld is affected\n",
				(long)domain->domid);
		} else if (!wrl_log_last_warning) {
			WRL_LOG(now, "rate limiting restarts\n");
		}
		wrl_log_last_warning = now.sec;
	}
}

void wrl_log_periodic(struct wrl_timestampt now)
{
	if (wrl_log_last_warning &&
	    (now.sec - wrl_log_last_warning) > WRL_LOGEVERY) {
		WRL_LOG(now, "not in force recently\n");
		wrl_log_last_warning = 0;
	}
}

void wrl_apply_debit_direct(struct connection *conn)
{
	if (!conn)
		/* some writes are generated internally */
		return;

	if (conn->transaction)
		/* these are accounted for when the transaction ends */
		return;

	if (!wrl_ntransactions)
		/* we don't conflict with anyone */
		return;

	wrl_apply_debit_actual(conn->domain);
}

void wrl_apply_debit_trans_commit(struct connection *conn)
{
	if (wrl_ntransactions <= 1)
		/* our own transaction appears in the counter */
		return;

	wrl_apply_debit_actual(conn->domain);
}

const char *dump_state_connections(FILE *fp)
{
	const char *ret = NULL;
	unsigned int conn_id = 1;
	struct xs_state_connection sc;
	struct xs_state_record_header head;
	struct connection *c;

	list_for_each_entry(c, &connections, list) {
		head.type = XS_STATE_TYPE_CONN;
		head.length = sizeof(sc);

		sc.conn_id = conn_id++;
		sc.pad = 0;
		memset(&sc.spec, 0, sizeof(sc.spec));
		if (c->domain) {
			sc.conn_type = XS_STATE_CONN_TYPE_RING;
			sc.spec.ring.domid = c->id;
			sc.spec.ring.tdomid = c->target ? c->target->id
						: DOMID_INVALID;
			sc.spec.ring.evtchn = c->domain->port;
		} else {
			sc.conn_type = XS_STATE_CONN_TYPE_SOCKET;
			sc.spec.socket_fd = c->fd;
		}

		ret = dump_state_buffered_data(NULL, c, &sc);
		if (ret)
			return ret;
		head.length += sc.data_in_len + sc.data_out_len;
		head.length = ROUNDUP(head.length, 3);
		if (fwrite(&head, sizeof(head), 1, fp) != 1)
			return "Dump connection state error";
		if (fwrite(&sc, offsetof(struct xs_state_connection, data),
			   1, fp) != 1)
			return "Dump connection state error";
		ret = dump_state_buffered_data(fp, c, NULL);
		if (ret)
			return ret;
		ret = dump_state_align(fp);
		if (ret)
			return ret;

		ret = dump_state_watches(fp, c, sc.conn_id);
		if (ret)
			return ret;
	}

	return ret;
}

static const char *dump_state_special_node(FILE *fp, const char *name,
					   const struct node_perms *perms)
{
	struct xs_state_record_header head;
	struct xs_state_node sn;
	unsigned int pathlen;
	const char *ret;

	pathlen = strlen(name) + 1;

	head.type = XS_STATE_TYPE_NODE;
	head.length = sizeof(sn);

	sn.conn_id = 0;
	sn.ta_id = 0;
	sn.ta_access = 0;
	sn.perm_n = perms->num;
	sn.path_len = pathlen;
	sn.data_len = 0;
	head.length += perms->num * sizeof(*sn.perms);
	head.length += pathlen;
	head.length = ROUNDUP(head.length, 3);
	if (fwrite(&head, sizeof(head), 1, fp) != 1)
		return "Dump special node error";
	if (fwrite(&sn, sizeof(sn), 1, fp) != 1)
		return "Dump special node error";

	ret = dump_state_node_perms(fp, perms->p, perms->num);
	if (ret)
		return ret;

	if (fwrite(name, pathlen, 1, fp) != 1)
		return "Dump special node path error";

	ret = dump_state_align(fp);

	return ret;
}

const char *dump_state_special_nodes(FILE *fp)
{
	const char *ret;

	ret = dump_state_special_node(fp, "@releaseDomain",
				      &dom_release_perms);
	if (ret)
		return ret;

	ret = dump_state_special_node(fp, "@introduceDomain",
				      &dom_introduce_perms);

	return ret;
}

void read_state_connection(const void *ctx, const void *state)
{
	const struct xs_state_connection *sc = state;
	struct connection *conn;
	struct domain *domain, *tdomain;

	if (sc->conn_type == XS_STATE_CONN_TYPE_SOCKET) {
#ifdef NO_SOCKETS
		barf("socket based connection without sockets");
#else
		conn = new_connection(&socket_funcs);
		if (!conn)
			barf("error restoring connection");
		conn->fd = sc->spec.socket_fd;
#endif
	} else {
		domain = introduce_domain(ctx, sc->spec.ring.domid,
					  sc->spec.ring.evtchn, true);
		if (!domain)
			barf("domain allocation error");

		conn = domain->conn;

		/*
		 * We may not have been able to restore the domain (for
		 * instance because it revoked the Xenstore grant). We need
		 * to keep it around to send @releaseDomain when it is
		 * dead. So mark it as ignored.
		 */
		if (!domain->port || !domain->interface)
			ignore_connection(conn, XENSTORE_ERROR_COMM);

		if (sc->spec.ring.tdomid != DOMID_INVALID) {
			tdomain = find_or_alloc_domain(ctx,
						       sc->spec.ring.tdomid);
			if (!tdomain)
				barf("target domain allocation error");
			talloc_reference(domain->conn, tdomain->conn);
			domain->conn->target = tdomain->conn;
		}
	}

	conn->conn_id = sc->conn_id;

	read_state_buffered_data(ctx, conn, sc);
}

/*
 * Local variables:
 *  mode: C
 *  c-file-style: "linux"
 *  indent-tabs-mode: t
 *  c-basic-offset: 8
 *  tab-width: 8
 * End:
 */
