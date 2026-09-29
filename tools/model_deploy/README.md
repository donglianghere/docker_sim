# 真机模型部署工具

把训练好的模型部署到机载机（NX01 / NX02）的全套工具。四个文件放一起，拷到哪都能用。

| 文件 | 作用 |
|---|---|
| `best.pt` | 要部署的模型（换模型就替换这个文件） |
| `deploy_model.sh` | 主脚本：导出 → 上传 → 编译 engine → 验证 |
| `setup_ssh.sh` | 配 SSH 免密（主脚本的前提，一般只需跑一次） |
| `README.md` | 本说明 |

## 用法

```bash
cd ~/ai_uav/tools/model_deploy
./setup_ssh.sh        # 第一次用，或飞机重装过系统后跑一次（要输密码）
./deploy_model.sh     # 之后每次换模型只跑这条
```

约 15 分钟（编译 engine 占 13 分钟，别中断）。跑完把节点重起一下就生效，**别的什么都不用做**。

## 参数（一般都不用填）

- `--host 192.168.2.101` — 换飞机，默认 `192.168.2.102`
- `--pt 别的.pt` — 换模型，默认 `best.pt`
- `--name 新名字` — 换产物名，默认用 `.pt` 的文件名

## 文件去向

- 模型放这儿：`tools/model_deploy/best.pt`
- 结果在飞机上：`/home/nvidia/ai_uav/docker_sim/models/best_960.engine`（容器内 `/models/best_960.engine`）
- 中间文件（onnx / names）自动删，两边都不留

## 例子

```
$ ./deploy_model.sh
[1/6] 类别=[StoveCabinet, Campfire]  imgsz=960
[2/6] 导出 ONNX (nms=True)
[3/6] [1,3,960,960] -> [1,300,6]  ✓
[4/6] 上传 + md5 一致
[5/6] Engine built in 745 sec
[6/6] 容器内验证 ✓
完成: /models/best_960.engine (23M)
```

## 出问题了

- **要密码 / 连不上** → 先跑 `./setup_ssh.sh`；飞机重装过系统的话它会提示清 host key
- **其它** → 看 `docker_real/ai_uav/docs/YOLO模型上机部署操作说明.md`
