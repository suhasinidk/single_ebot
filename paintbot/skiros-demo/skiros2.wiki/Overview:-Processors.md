# Processors
Processors defines the policy that a *compound* skill uses to access its children. We define the following processors:
* **Serial** (->): The default processor. Process children in sequence until all succeed. Restarts finished (succeeded/failed) skills. Returns on first occurrence of a running of failed skill.
* **SerialStar** (->*): Like serial, but keeps memory of which nodes previously succeeded and does not tick them again. The memory index is reset on failure, success or preemption.
* **Selector** (?): Process children in sequence until one succeeds, ignoring failures. Returns on first occurrence of a running or successful skill.
* **SelectorStar** (?*): Like Selector, but keeps memory of which nodes previously succeeded and do not tick them again. The memory index is reset on failure, success or preemption.
* **ParallelFf** (Parallel First Fail): Process children in parallel until all succeed. Stop all processes if a child fails.
* **ParallelFs** (Parallel First Stop): Process children in parallel until one succeed. Stop all processes if a child finishes (succeeded/fail).

# Decorators
Decorators modify the return state of a Processor. The available decorator is currently:
* **NoFail**: Returns only running or success. Failure state is converted in Success state.