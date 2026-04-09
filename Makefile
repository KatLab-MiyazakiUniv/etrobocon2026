MAKEFILE_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# 使い方
help:
	@echo ビルドする
	@echo " $$ make build"
	@echo 走行を開始する\(実機限定\)
	@echo " $$ make start"
	@echo 指定ファイルをフォーマットする
	@echo " $$ make format FILES=<ディレクトリ名>/<ファイル名>.cpp"
	@echo すべての変更ファイルをフォーマットする
	@echo " $$ make format"
	@echo フォーマットチェックをする
	@echo " $$ make format-check"
	@echo テストをビルドする
	@echo " $$ make test-build"
	@echo テストを実行する
	@echo " $$ make test-exec"
	@echo テスト用の'test_build'ディレクトリを削除する
	@echo " $$ make clean"
	@echo 環境が変わっている場合のみ test_build ディレクトリを削除する
	@echo " $$ make smart-clean"
	@echo 必要があればtest_buildディレクトリを削除し, テストをビルドして実行する
	@echo " $$ make test"

## 実行関連 ##
.PHONY: build
build:
	cd $(MAKEFILE_PATH)../ && make img=etrobocon2026

# 実機の場合、走行を開始する
start:
	cd $(MAKEFILE_PATH)../ && make start

## 開発関連 ##
# ファイルにclang-formatを適用する
format:
# 指定ファイルがある場合、そのファイルにclang-formatを適用する
ifdef FILES
	clang-format -i -style=file $(FILES)
	@ echo "フォーマットを適用しました: $(FILES)"
# ない場合、変更されたファイルのうち、cpp、hファイルにclang-formatを適用する
else
# 変更されたファイルと未追跡のファイル (新規追加など) の両方を検出
	@{ \
        CANDIDATE_FILES=$$( (git diff origin/main --name-only; git ls-files --others --exclude-standard) | grep -E '\.cpp$$|\.h$$' || true ); \
        ACTUALLY_FORMATTED_FILES=""; \
        FORMATTED_COUNT=0; \
        if [ -n "$$CANDIDATE_FILES" ]; then \
            for FILE_PATH in $$CANDIDATE_FILES; do \
                REPLACEMENTS_XML=$$(clang-format -style=file --output-replacements-xml "$$FILE_PATH" 2>/dev/null); \
                if echo "$$REPLACEMENTS_XML" | grep -q "<replacement "; then \
                    clang-format -i -style=file "$$FILE_PATH"; \
                    if [ -z "$$ACTUALLY_FORMATTED_FILES" ]; then \
                        ACTUALLY_FORMATTED_FILES="$$FILE_PATH"; \
                    else \
                        ACTUALLY_FORMATTED_FILES="$${ACTUALLY_FORMATTED_FILES}\n$$FILE_PATH"; \
                    fi; \
                    FORMATTED_COUNT=$$(($$FORMATTED_COUNT + 1)); \
                fi; \
            done; \
            if [ $$FORMATTED_COUNT -gt 0 ]; then \
                echo "以下のファイルにフォーマットを適用しました ($$FORMATTED_COUNT 件):"; \
                printf "%b\n" "$$ACTUALLY_FORMATTED_FILES"; \
            else \
                echo "検査したファイルは全てフォーマット済みでした。適用された変更はありません。"; \
            fi; \
        else \
            echo "フォーマットをチェックする対象の .cpp または .h ファイルがありません。"; \
        fi \
    }
endif

format-check:
	find ./modules -type f \( -name "*.cpp" -o -name "*.h" \) | xargs clang-format --dry-run --WerrorMAKEFILE_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

## テスト関連 ##
# テストのビルドディレクトリが存在しない場合は作成する
test-build:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. && make

# テストを実行する
test-exec:
	@if [ ! -f $(MAKEFILE_PATH)test_build/etrobocon2026_test ]; then \
		echo "テスト実行ファイルが見つかりません。まずビルドを実行してください。"; \
		echo " $$ make test-build"; \
		exit 1; \
	fi
	cd $(MAKEFILE_PATH)test_build && ./etrobocon2026_test

# テストをビルドして実行する
test: smart-clean test-build test-exec

# test_build ディレクトリを完全に削除する
clean:
	@if [ -d $(MAKEFILE_PATH)test_build ]; then \
		rm -rf $(MAKEFILE_PATH)test_build; \
		echo "'test_build/' ディレクトリを削除しました。"; \
	else \
		echo "'test_build/' ディレクトリは既に存在しません。"; \
	fi

# 実行環境が変更されている場合にのみ 'test_build' を削除する
smart-clean:
	@if [ -d $(MAKEFILE_PATH)test_build ]; then \
		if [ -f "$(MAKEFILE_PATH)test_build/Makefile" ]; then \
			CMAKE_SOURCE_DIR=`grep -E "^CMAKE_SOURCE_DIR[[:space:]]*=" $(MAKEFILE_PATH)test_build/Makefile | cut -d= -f2 | xargs`; \
			CMAKE_SOURCE_DIR_REAL=`readlink -f "$$CMAKE_SOURCE_DIR"`; \
			CURRENT_DIR_REAL=`readlink -f "$$(pwd)"`; \
			echo "[DEBUG] CMAKE_SOURCE_DIR: '$$CMAKE_SOURCE_DIR_REAL'"; \
			echo "[DEBUG] CURRENT_DIR    : '$$CURRENT_DIR_REAL'"; \
			if [ "$$CMAKE_SOURCE_DIR_REAL" != "$$CURRENT_DIR_REAL" ]; then \
				echo "[LOG] 実行環境の変更が検出されたため 'test_build/' を削除します。"; \
				rm -rf $(MAKEFILE_PATH)test_build; \
			else \
				echo "[LOG] 実行環境は変更されていません。"; \
			fi; \
		fi; \
	else \
		echo "'test_build/' ディレクトリは既に存在しません。"; \
	fi