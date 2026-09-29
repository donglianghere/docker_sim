#!/usr/bin/env bash
# 配置到机载机的 SSH 免密登录（deploy_model.sh 等脚本都依赖它）
#
# 用法:
#   ./setup_ssh.sh                          # 给默认的 101、102 配
#   ./setup_ssh.sh 192.168.2.100            # 指定主机
#   ./setup_ssh.sh --user hx 192.168.2.100  # 指定用户名
#
# 幂等：已经配好的直接跳过，不会重复往 authorized_keys 里追加。
# 每台没配过的机器需要你输一次密码（之后就不用了）。
set -uo pipefail

USER_NAME="nvidia"
HOSTS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --user) USER_NAME="$2"; shift 2;;
        -h|--help) sed -n '2,12p' "$0"; exit 0;;
        *) HOSTS+=("$1"); shift;;
    esac
done
[[ ${#HOSTS[@]} -gt 0 ]] || HOSTS=("192.168.2.101" "192.168.2.102")

KEY="$HOME/.ssh/id_ed25519"

# ---- 1. 本地密钥 ----
if [[ -f "$KEY" ]]; then
    echo "本地密钥已存在: $KEY"
else
    echo "本地没有密钥，正在生成（无口令，供脚本免交互使用）..."
    ssh-keygen -t ed25519 -N "" -C "$(whoami)@$(hostname)" -f "$KEY"
fi

ok=(); failed=(); skipped=()
for H in "${HOSTS[@]}"; do
    T="${USER_NAME}@${H}"
    echo
    echo "---------- $T ----------"

    # ---- 2. 已经免密就跳过 ----
    if timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=5 "$T" "true" 2>/dev/null; then
        echo "  已经免密，跳过"
        skipped+=("$T"); continue
    fi

    # ---- 3. 主机可达性 ----
    if ! timeout 6 bash -c "echo > /dev/tcp/${H}/22" 2>/dev/null; then
        echo "  ✗ 连不上 ${H}:22（关机？没连网？）"
        failed+=("$T"); continue
    fi

    # ---- 4. host key 变过就先清掉旧的 ----
    #    机载机重装/整盘回退之后 host key 会变，不清会报
    #    REMOTE HOST IDENTIFICATION HAS CHANGED 然后拒绝连接
    if ssh-keygen -F "$H" >/dev/null 2>&1; then
        probe=$(timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=5 "$T" "true" 2>&1)
        if grep -q "HOST IDENTIFICATION HAS CHANGED" <<<"$probe"; then
            echo "  ! 该主机的 host key 变了（多半是重装或系统回退过）"
            read -r -p "    清掉 known_hosts 里的旧记录并继续？[y/N] " a
            [[ "$a" == y || "$a" == Y ]] || { failed+=("$T"); continue; }
            ssh-keygen -R "$H" >/dev/null 2>&1
        fi
    fi

    # ---- 5. 推公钥（这一步要输密码）----
    echo "  推送公钥，请输入 $T 的密码："
    if ssh-copy-id -o StrictHostKeyChecking=accept-new -i "${KEY}.pub" "$T"; then
        if timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=5 "$T" "true" 2>/dev/null; then
            echo "  ✓ 免密配置成功"
            ok+=("$T")
        else
            echo "  ✗ 公钥推过去了但仍然要密码（看看对端 ~/.ssh 权限："
            echo "     .ssh 应为 700，authorized_keys 应为 600）"
            failed+=("$T")
        fi
    else
        echo "  ✗ ssh-copy-id 失败"
        failed+=("$T")
    fi
done

echo
echo "=========================================="
[[ ${#ok[@]}      -gt 0 ]] && echo "本次配好:   ${ok[*]}"
[[ ${#skipped[@]} -gt 0 ]] && echo "本来就好:   ${skipped[*]}"
[[ ${#failed[@]}  -gt 0 ]] && echo "失败:       ${failed[*]}"
echo "=========================================="
[[ ${#failed[@]} -eq 0 ]]
