# 使い方: etrobocon2026/ ルートから ./scripts/check_type.sh [<Area名> <L|R>]
# 例: ./scripts/check_type.sh LineTrace L

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BINARY="$SCRIPT_DIR/CheckType"
SOURCE="$SCRIPT_DIR/CheckType.cpp"

# ソースが新しければ再コンパイル
# if [ ! -f "$BINARY" ] || [ "$SOURCE" -nt "$BINARY" ]; then
  echo "コンパイル中..."
  g++ -std=c++17 -o "$BINARY" "$SOURCE"
# fi

# datafiles/ への相対パスが正しくなるように etrobocon2026/ ルートから実行
cd "$SCRIPT_DIR/.."

exec "$BINARY" "$@"
