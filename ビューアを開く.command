#!/bin/zsh
# 確認ビューアを開く（このフォルダを簡易サーバーで配信してブラウザで表示）
cd "$(dirname "$0")"
open "http://localhost:8768/viewer.html?dir=output/glb_web"
python3 -m http.server 8768
