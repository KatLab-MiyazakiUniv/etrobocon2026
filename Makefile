MAKEFILE_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

DOCKER_IMAGE   := kat_etrobo2026:arm64
DOCKER_WORKDIR := /RasPike-ART/sdk/workspace/etrobocon2026
DOCKER_MOUNT   := -v $(MAKEFILE_PATH):$(DOCKER_WORKDIR)

# 全システムの使い方
help:
	@echo 全システムをビルドする
	@echo " $$ make build"
	@echo 走行システムをビルドする
	@echo " $$ make build-running"
	@echo 撮影システムをビルドする
	@echo " $$ make build-camera"
	@echo 走行を開始する\(実機限定\)
	@echo " $$ make start"
	@echo 走行システムを開始する
	@echo " $$ make start-running"
	@echo 撮影システムを開始する
	@echo " $$ make start-camera"
	@echo 指定ファイルをフォーマットする
	@echo " $$ make format FILES=<ディレクトリ名>/<ファイル名>.cpp"
	@echo すべての変更ファイルをフォーマットする
	@echo " $$ make format"
	@echo フォーマットチェックをする
	@echo " $$ make format-check"
	@echo 全システムのテストをビルドする
	@echo " $$ make test-build"
	@echo 走行システムのテストをビルドする
	@echo " $$ make test-build-running"
	@echo 撮影システムのテストをビルドする
	@echo " $$ make test-build-camera"
	@echo 全システムのテストを実行する
	@echo " $$ make test-exec"
	@echo 走行システムのテストを実行する
	@echo " $$ make test-exec-running"
	@echo 撮影システムのテストを実行する
	@echo " $$ make test-exec-camera"
	@echo 全システムのテストをビルドして実行する
	@echo " $$ make test"
	@echo 走行システムのテストをビルドして実行する
	@echo " $$ make test-running"
	@echo 撮影システムのテストをビルドして実行する
	@echo " $$ make test-camera"
	@echo テスト用の'test_build'ディレクトリを削除する
	@echo " $$ make clean"
	@echo 環境が変わっている場合のみ test_build ディレクトリを削除する
	@echo " $$ make smart-clean"
	@echo Dockerイメージをビルドする
	@echo " $$ make docker-build"
	@echo Dockerコンテナを起動する
	@echo " $$ make docker-run"
	@echo UID/GIDを指定してDockerコンテナを起動する\(権限問題が起きた場合\)
	@echo " $$ make docker-run-user"

## 実行関連 ##
.PHONY: build
build: build-running build-camera

# 走行システムをビルドする
build-running:
	cd $(MAKEFILE_PATH)../ && make img=etrobocon2026 -j5

# 撮影システムをビルドする
build-camera:
	cd $(MAKEFILE_PATH)camera_server && make -f Makefile.camera -j5

# 実機の場合、走行を開始する
start: start-camera start-running

# 走行システムを開始する
start-running:
	cd $(MAKEFILE_PATH)../ && make start

# 撮影システムを開始する
start-camera:
	cd $(MAKEFILE_PATH)camera_server && ./camera_app

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
	find ./modules -type f \( -name "*.cpp" -o -name "*.h" \) | xargs clang-format --dry-run --Werror

## テスト関連 ##
# 全システムのテストをビルドする
test-build:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. && make

# 走行システムのテストをビルドする
test-build-running:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. -DENABLE_CAMERA_TESTS=OFF && make running_test

# 撮影システムのテストをビルドする
test-build-camera:
	@mkdir -p $(MAKEFILE_PATH)test_build
	cd $(MAKEFILE_PATH)test_build && cmake .. -DENABLE_CAMERA_TESTS=ON && make camera_server_test

# 全システムのテストを実行する
test-exec: test-exec-running test-exec-camera

# 走行システムのテストを実行する
test-exec-running:
	@if [ ! -f $(MAKEFILE_PATH)test_build/running_test ]; then \
		echo "テスト実行ファイルが見つかりません。まずビルドを実行してください。"; \
		echo " $$ make test-build-running"; \
		exit 1; \
	fi
	cd $(MAKEFILE_PATH)test_build && ./running_test

# 撮影システムのテストを実行する
test-exec-camera:
	@if [ ! -f $(MAKEFILE_PATH)test_build/camera_server_test ]; then \
		echo "テスト実行ファイルが見つかりません。まずビルドを実行してください。"; \
		echo " $$ make test-build-camera"; \
		exit 1; \
	fi
	cd $(MAKEFILE_PATH)test_build && ./camera_server_test

# 全システムのテストをビルドして実行する
test: smart-clean test-build test-exec

# 走行システムのテストをビルドして実行する
test-running: smart-clean test-build-running test-exec-running

# 撮影システムのテストをビルドして実行する
test-camera: smart-clean test-build-camera test-exec-camera

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

## Docker関連 ##
docker-build:
	docker buildx build --platform linux/arm64 -t $(DOCKER_IMAGE) .

docker-run:
	docker run -it --rm \
		$(DOCKER_MOUNT) \
		$(DOCKER_IMAGE) bash

docker-run-user:
	docker run -it --rm \
		--user $$(id -u):$$(id -g) \
		$(DOCKER_MOUNT) \
		$(DOCKER_IMAGE) bash