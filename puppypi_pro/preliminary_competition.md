> You can pass any number of arguments you like to the trigger.
>
> There is one important limitation to this approach: every callback function triggered by the state transition must be able to handle all of the arguments.
>
> https://github.com/pytransitions/transitions#passing-data

将线程池提交后返回的 future object 传入 trigger, 会导致 trigger 回调的函数不能处理 future 这个入参而回调失败. 因此使用 `EventData` 解决 trigger 入参问题.

如果上述方法不行, 就考虑 `transitions` 回调的策略是否与 `concurrent.futures` 的运行逻辑以及 _Python_ 实例化时处理实例方法上的 decorator 的逻辑有冲突.

> The callable resolution is done in `Machine.resolve_callable`.
> 
> This method can be overridden in case more complex callable resolution strategies are required.
> 
> https://github.com/pytransitions/transitions#resolution
> 
> https://segmentfault.com/a/1190000004420386
> 
> https://python3-cookbook.readthedocs.io/zh_CN/latest/c09/p08_define_decorators_as_part_of_class.html