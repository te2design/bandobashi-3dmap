#!/bin/zsh
# 阪東橋 3D マップを作り直す（data/ にある元データから。消えていればダウンロードし直す）
set -e
cd "$(dirname "$0")"
BLENDER=/Applications/Blender.app/Contents/MacOS/Blender
TMP=${TMPDIR:-/tmp}/bandobashi_raw
mkdir -p "$TMP"

if [ ! -f data/osm_raw.json ]; then
  echo "OSM を取得…"
  BB=$(tr -d '\n' < data/bbox.txt)
  Q="[out:json][timeout:180];(way[\"highway\"]($BB);way[\"building\"]($BB);relation[\"building\"]($BB);way[\"railway\"]($BB);way[\"waterway\"]($BB);way[\"natural\"]($BB);relation[\"natural\"=\"water\"]($BB);way[\"landuse\"]($BB);way[\"leisure\"]($BB);relation[\"leisure\"]($BB);relation[\"landuse\"]($BB);relation[\"waterway\"=\"riverbank\"]($BB);node[\"highway\"~\"traffic_signals|crossing|stop\"]($BB););out body;>;out skel qt;"
  curl -s -m 300 --data-urlencode "data=$Q" https://overpass-api.de/api/interpreter -o data/osm_raw.json
fi
if [ ! -f data/buildings.jsonl ]; then
  echo "PLATEAU 建物を取得（約1GB・作業後は一時フォルダに置いたまま）…"
  mkdir -p "$TMP/plateau_bldg"
  (cd "$TMP/plateau_bldg" && xargs -n1 -P4 curl -s -O < "$OLDPWD/data/plateau_bldg_urls.txt")
  python3 -I scripts/prep_plateau.py "$TMP/plateau_bldg" data/buildings.jsonl
fi
if [ ! -f data/dem.json ]; then
  echo "標高タイルを取得…"
  python3 -I scripts/prep_dem.py "$TMP/dem_tiles" data/dem.json
fi

echo "Blender で組み立て…"
"$BLENDER" -b --factory-startup --python scripts/build.py -- \
  "$PWD/data/osm_raw.json" "$PWD/data/dem.json" "$PWD/data/buildings.jsonl" "$PWD/output" > output/blender_stdout.txt 2>&1
cat output/build_log.txt

echo "検査…"
"$BLENDER" -b output/bandobashi_3dmap.blend --python scripts/check.py 2>&1 | grep "検査"

echo "Web 用の軽量版…"
mkdir -p output/glb_web
for f in output/glb/*.glb; do
  b=$(basename "$f")
  npx -y @gltf-transform/cli@4.5.1 weld "$f" "$TMP/_w.glb" > /dev/null
  npx -y @gltf-transform/cli@4.5.1 quantize "$TMP/_w.glb" "output/glb_web/$b" --quantize-position 16 --quantize-normal 10 > /dev/null
done
ls -la output/glb_web

echo "公開用フォルダ（site/）を組み立て…"
mkdir -p site/glb  # 上書きで更新（古いファイルは消さない）
cp output/glb_web/*.glb site/glb/
cp output/buildings_index.json output/lanes.json site/
sed -e 's|data-dir="output/glb_web"|data-dir="glb"|' -e 's|data-index="output/buildings_index.json"|data-index="buildings_index.json"|' viewer.html > site/index.html
cat > site/_headers <<'H'
/glb/*
  Content-Type: model/gltf-binary
  Cache-Control: public, max-age=3600
  Access-Control-Allow-Origin: *
/*.json
  Access-Control-Allow-Origin: *
H
du -sh site
echo "完了"
