# 开发文档

## 一、文档说明

- 在使用K210的过程中，存在各种各样的问题。而如何将K210接入局域网，进行图像传输，是这次项目的核心，但是，由于k210的核心以及源码部分不予开源，而我们采用的W5500并不是在官方店铺进行购买的，如下图所示。所以次次项目就是为了打通k210的spi，使其能够正常与W5500进行通信。

  ![](inc/img/w5500img.jpg)
  
  - 目前采用建立 tcp 的方式进行图像传输，在现在的模式下，帧率大约在 10 帧左右



## 二、文档结构

- **img** 为开发过程中的截屏文件
- **inc** 为资料文件
- **src** 为核心代码



## 三、开发人员

- *Liu-Jiahao*



## 四、项目进度

- 修改人：*Liu-Jiahao*
- 时间：2023-6-7
- 修改内容：
  - 添加 **项目文件** 
  - 配置相关参数设置



- 修改人：*Liu-Jiahao*
- 时间： 2023-6-8
- 修改内容：
  - 增加 **mySocket** 类 和 **myPing** 类，书写相关函数
  - 解决 k210 与 windows ping不同的情况
    - 将网关设置为与windows相同即可
  - 成功实现 k210、windows、树莓派的局域网搭建，并且互 ping 成功



- 修改人：*Liu-Jiahao*
- 时间： 2023-6-9
- 修改内容：
  - 完善 **mySocket** 类 和 **myPing** 类，书写相关函数
  - 代码架构基本写完，但是仍然存在 bug



- 修改人：*Liu-Jiahao*
- 时间：2023-7-9
- 修改内容：
  - 完善代码中存在的 bug
  - 完美解决 w5500 ping 主机的问题



- 修改人：*Liu-Jiahao*
- 时间：2023-7-10
- 修改内容：
  - 在 **src** 目录中添加 **Server** 和 **Client** 端
  - 在 **Server** 中为电脑环境，需要装有 python、python-opencv 支持
  - 在 **Client** 中为 K210 环境，运行 Maixpy
  - 基本完成此项目，后续会进行代码优化等工作



## 五、仓库地址

- GitHub: [Jiahao-Liu29/W5500_Get (github.com)](https://github.com/Jiahao-Liu29/W5500_Get)
- Gitee: [刘佳豪/W5500_Get (gitee.com)](https://gitee.com/liu-jiahaohappy/W5500_Get)



## 六、版权说明

- 资料以及源代码均来自 **野火旗舰店**，其资料链接如下
  - [野火产品资料下载中心 — 野火产品资料下载中心 文档 (embedfire.com)](https://doc.embedfire.com/products/link/zh/latest/index.html)

