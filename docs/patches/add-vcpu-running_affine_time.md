## Patch: rrd3-add-vcpu-running_affine_time

### Purpose

Base dependency of CP-53669 (Summarize non-affine vCPU time for each Domain):

For this, we need the amount of time a vCPU does not run on the NUMA node(s)
it should preferably run.

More precise: The amount of time a vCPU is running, but not running on the NUMA
nodes it shall preferably run due to the use of vCPU-affinity.

Running non-affine can only happen when the vCPU has a soft-affinty mask that
is not equal to all pCPUs. The soft-affinity mask is a bitmask with one bit
set for each pCPU the vCPU is affine to.

- When a vCPU is hard-pinned, the vCPU is 100% affine, thus hard-pinning does
  not need to be considered in this context.

- While it is possible to combine hard-pinning with soft-affinity this metric
  shall only be concerned with soft-affinity, for the same reason that
  hard-pinning is an absolute contraint that does not make sense to measure.

- Not considering hard-pinning for this metric also allows to test this
  the affine/non-affine metric by setting constraints using hard-pinning.

The system memory for domain (more precise: the vCPU) is be allocated on
NUMA nodes with the pCPUs that the vCPU will be affine to.

### Implementation

Therefore, to collect the needed time running affine or non-affine, we do not
need to inspect where the memory of the domain is allocated, but only check if
th vCPU's processor's bit is set in the vCPU's soft-affinty mask.

This simplicity has a number of benefits:

- It requires the least amount of code and data to get the needed information
- It incurs the least amount of overhead

### Design

Using the pull archtecture is the standard for most monitoring solutions.
Prometheus, SNMP and JMX are pull based, and even Nagios often operates
via a pull model.

Likewise, the XAPI toolstack also uses the pull model, where metrics plugins
are called by XCP-RRDD which then pull the raw metrics data from raw metrics
data exporters like other programs, procfs files or using Hypercalls.

### Overhead discussion

Understandably, there is the desire to disable rarely needed metrics by default,
but the way Xen and the XAPI toolstack currently implement CPU time metrics is
that the metrics exporters are always ready to provide the raw data for these metrics.

As the raw vCPU data is implemented by returning monotonically incrementing values
from the Xen Hypervisor to Dom0 (and for some vCPU metrics also to the DomU using
shared guest memory) the code to track the raw data in Xen cannot be dynamically
enabled or disabled: The exporters are always available.

Enabling or disabling metrics is only done in the Xapi toolstack, and "disabling"
a metric only means to to not accumulating RRD data for disabled metrics in RRDD.
But, long as the respective CPU plugin for XCP-RRDD is active and called by RRDD,
the CPU plugin simply asks the Hypervisor to return the collected data. The CPU
plugin for XCP-RRDD does not currently have an interface to enable or disable
metrics. See the implementation details below for Xen.

However, as VM exits and HW interrupts do not change the vCPU runstate,
changes of the vCPU runstate are sufficiently rare that the overhead to
collect data on vCPU runstate changes is sufficiently low. XCP-rrdd calls
its vCPU plugin at an interval of 5 seconds only so also the hypercall overhead
is minimal.

PS: Other metrics call programs in Dom0 at intervals and thereby incur
huge overheads compared to vCPU metrics, so overhead is not a problem.

Nonetheless, as the raw data collection is "always-on", the implementation
shall try to not jump extra hoops that it does not need.

### Implementation

The accounting of the affine/non-affine CPU time needs be done at two locations:

1. vcpu_runstate_change(), called by the scheduler when a vCPU runstate changes
2. vcpu_runstate_get(), called by hypercalls that read the current vCPU state.

This accounting is implemented as follows:

1. In vcpu_runstate_change(), when the previous vCPU runstate was RUNSTATE_running,
   the consumed vCPU runtime is added to to the running affine or non-affine time
   (depending on if the processor the vCPU's used was in the vCPU's soft-affinty mask)
   and updated in the vCPU struct.

   This is a very fast bit-wise AND instruction and adding a flag and checking
   before running the AND instruction would create more CPU branches and possibly
   fetches from L2 or L3 cache (or even RAM) and therefore adding another check
   if this AND instruction shall run is not justified and can even cause much
   larger overheads and more complexity and infrastructure in Xen to enable and
   disable metrics, even if the counting the vCPU time would be redefined to be
   not continously incremented.

2. In vcpu_runstate_get(), when the current vCPU runstate is RUNSTATE_running,
   the vCPU runtime consumed so far since the vCPU was created is added to
   the time cumulated in vcpu_runstate_change() (above). The result may not be
   written to the vCPU struct but is only returned to the caller as a temporary
   snapshot value that is defined as nanoseconds running affine or non-affine
   since vCPU creation.

   This is also a very fast operation that does not justfiy adding any flags
   on wether to run it or not. This would complicate the code without any
   benefits.
