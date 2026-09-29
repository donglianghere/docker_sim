# tools

真机侧的两套离线工具，2026-09-29 从 `~/ai_uav/tools/` 复制进来纳入版本控制。

| 目录 | 作用 |
|---|---|
| `model_deploy/` | 训练好的 YOLO 模型部署到机载机（导出 → 上传 → 编译 engine → 验证） |
| `calib_tools/` | 相机标定（标定板生成、采图、单目标定、多组结果合并） |

## ⚠️ 这是一份**副本**，不是唯一一份

原件在 `~/ai_uav/tools/`，那个位置是 `docker_sim/` 和 `docker_real/` 的共同上级，
两边共用；各 README 和 `docker_real/ai_uav/docs/YOLO模型上机部署操作说明.md` 里写的
操作路径都是 `cd ~/ai_uav/tools/...`，所以原件不能搬走。

**改动请改原件 `~/ai_uav/tools/`，改完同步回这里再提交**，否则两份会悄悄分叉。
同步：

```bash
cp -a ~/ai_uav/tools/. ~/ai_uav/docker_sim/tools/
```

`model_deploy/best.pt`(20 MB) 和 `best_960.onnx`(37 MB) 是模型权重二进制，
一并进了 git——换模型时注意每换一次仓库就永久多一份历史体积。
