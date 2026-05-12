# 使い方: etrobocon2026/ ルートから ./scripts/merge_commands.sh [<Area名> <L|R>]
# 例: ./scripts/merge_commands.sh LineTrace L

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BINARY="$SCRIPT_DIR/CommandMerger"
SOURCE="$SCRIPT_DIR/CommandMerger.cpp"

# ソースが新しければ再コンパイル
if [ ! -f "$BINARY" ] || [ "$SOURCE" -nt "$BINARY" ]; then
  echo "コンパイル中..."
  g++ -std=c++17 -o "$BINARY" "$SOURCE"
fi

# datafiles/ への相対パスが正しくなるように etrobocon2026/ ルートから実行
cd "$SCRIPT_DIR/.."

exec "$BINARY" "$@"
