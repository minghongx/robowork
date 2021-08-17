X Window System 从本地客户端转发画面到远程服务器需要满足三个条件:
- [Passwordless SSH Access](https://www.raspberrypi.org/documentation/computers/remote-access.html#passwordless-ssh-access)
- 远程安装 X server, 启动时关闭 access control
- Pycharm 的 Run Configuration 上添加一个 env var DISPLAY=<服务器的 IP Address>:<服务器开放的端口, 常为 0.0>

X 转发的画面延迟非常高. 未来选择用 Wayland 替代 X.
> X is a brain-dead protocol. Why X isn't dead by now is astonishing.
 
> a free and open-source community-driven project with the aim of replacing the X Window System with a modern, secure simpler windowing system in Linux and other Unix-like operating systems.
> 
> From [Wikipedia](https://en.wikipedia.org/wiki/Wayland_(display_server_protocol))