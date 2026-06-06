# ====================================================================
# 【新設】共通（common）ライブラリターゲットの作成
# ====================================================================
# common直下のcpp（RealNetworkSystem.cppなど）を一箇所だけでビルドする
file(GLOB COMMON_SOURCES "${PROJECT_SOURCE_DIR}/common/*.cpp")
add_library(common_impl STATIC ${COMMON_SOURCES})
target_include_directories(common_impl PUBLIC
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/common/customs
)

# ====================================================================
# ## 走行システム ##
# ====================================================================

set(RUNNING_INCLUDE_DIRS
  ${RUNNING_MODULE_DIR}
  ${RUNNING_MODULE_DIR}/calculators
  ${RUNNING_MODULE_DIR}/API
  ${RUNNING_MODULE_DIR}/client
  ${RUNNING_MODULE_DIR}/motions
  ${RUNNING_MODULE_DIR}/motions/continuation_conditions
  ${RUNNING_TEST_DIR}
  ${RUNNING_TEST_DIR}/dummy
  ${RUNNING_TEST_DIR}/datafiles/logfiles
)

# ⚠️ 注意: ここから "${PROJECT_SOURCE_DIR}/common/*.cpp" を削除しました！
file(GLOB_RECURSE RUNNING_SOURCES
  "${RUNNING_MODULE_DIR}/*.cpp"
  "${RUNNING_TEST_DIR}/dummy/utils/*.cpp"
)

add_library(running_impl STATIC ${RUNNING_SOURCES})
target_include_directories(running_impl PUBLIC ${RUNNING_INCLUDE_DIRS})
# 走行用ライブラリに共通ライブラリをリンク
target_link_libraries(running_impl PUBLIC common_impl)

file(GLOB RUNNING_TEST_SOURCES "${RUNNING_TEST_DIR}/*Test.cpp")

add_executable(running_test ${RUNNING_TEST_SOURCES})
target_link_libraries(running_test
  PRIVATE
    running_impl
    common_impl # 👈 共通ライブラリを結合
    gtest_main
)
gtest_discover_tests(running_test)

# ====================================================================
# ## 撮影システム ##
# ====================================================================
option(ENABLE_CAMERA_TESTS "Build camera tests" ON)

if(ENABLE_CAMERA_TESTS)
  find_package(OpenCV REQUIRED)

  set(CAMERA_INCLUDE_DIRS
    ${CAMERA_MODULE_DIR}/camera
    ${CAMERA_MODULE_DIR}/utils
    ${CAMERA_MODULE_DIR}/server
    ${CAMERA_MODULE_DIR}/image_processors
    ${RUNNING_MODULE_DIR}/API
    ${RUNNING_TEST_DIR}/dummy
  )

  # ⚠️ 注意: ここからも "${PROJECT_SOURCE_DIR}/common/*.cpp" を削除しました！
  file(GLOB_RECURSE CAMERA_SOURCES
    "${CAMERA_MODULE_DIR}/camera/*.cpp"
    "${CAMERA_MODULE_DIR}/utils/*.cpp"
    "${CAMERA_MODULE_DIR}/server/*.cpp"
    "${CAMERA_MODULE_DIR}/image_processors/*.cpp"
    "${RUNNING_MODULE_DIR}/API/*.cpp"
  )

  add_library(camera_server_impl STATIC ${CAMERA_SOURCES})
  target_include_directories(camera_server_impl PUBLIC ${CAMERA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  # 撮影用ライブラリに共通ライブラリとOpenCVをリンク
  target_link_libraries(camera_server_impl PUBLIC common_impl ${OpenCV_LIBS})

  file(GLOB CAMERA_TEST_SOURCES "${CAMERA_TEST_DIR}/*Test.cpp")

  add_executable(camera_server_test ${CAMERA_TEST_SOURCES})
  target_link_libraries(camera_server_test
    PRIVATE
      camera_server_impl
      common_impl # 👈 共通ライブラリを結合
      gtest_main
      ${OpenCV_LIBS}
  )
  gtest_discover_tests(camera_server_test)
endif()















# # GoogleTest用のCMakeLists.txt
# cmake_minimum_required(VERSION 3.14)
# project(etrobocon2026)

# # C++標準とカバレッジ用フラグ
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# # デバッグビルド用のフラグ
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# # カバレッジレポート作成のオプション
# option(ENABLE_COVERAGE "Enable test coverage reporting" ON)
# if(ENABLE_COVERAGE)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
# endif()

# # GoogleTest のフェッチとビルド
# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
# )
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

# # プロジェクトのライブラリターゲットを作成
# set(RUNNING_MODULE_DIR ${PROJECT_SOURCE_DIR}/modules)
# set(RUNNING_TEST_DIR ${PROJECT_SOURCE_DIR}/tests)
# set(CAMERA_MODULE_DIR ${PROJECT_SOURCE_DIR}/camera_server/modules)
# set(CAMERA_TEST_DIR ${PROJECT_SOURCE_DIR}/camera_server/tests)

# # テスト設定
# enable_testing()
# include(GoogleTest)

# ## 走行システム ##

# # 必要なインクルードディレクトリの設定 (適宜登録)
# set(RUNNING_INCLUDE_DIRS
#   ${RUNNING_MODULE_DIR}
#   ${RUNNING_MODULE_DIR}/calculators
#   ${RUNNING_MODULE_DIR}/API
#   ${RUNNING_MODULE_DIR}/client
#   ${RUNNING_MODULE_DIR}/motions
#   ${RUNNING_MODULE_DIR}/motions/continuation_conditions
#   ${PROJECT_SOURCE_DIR}/common
#   ${PROJECT_SOURCE_DIR}/common/customs
#   ${RUNNING_TEST_DIR}
#   ${RUNNING_TEST_DIR}/dummy
#   ${RUNNING_TEST_DIR}/datafiles/logfiles
# )

# # ソースファイルの登録 (適宜登録)
# file(GLOB_RECURSE RUNNING_SOURCES
#   "${RUNNING_MODULE_DIR}/*.cpp"
#   "${RUNNING_TEST_DIR}/dummy/utils/*.cpp"
#   "${PROJECT_SOURCE_DIR}/common/*.cpp"
# )

# # ライブラリターゲットを作成
# add_library(running_impl STATIC ${RUNNING_SOURCES})
# target_include_directories(running_impl PUBLIC ${RUNNING_INCLUDE_DIRS})

# # tests ディレクトリ内の *Test.cpp をすべてテストソースとして登録
# file(GLOB RUNNING_TEST_SOURCES
#   "${RUNNING_TEST_DIR}/*Test.cpp"
# )

# # テスト実行可能ファイルの作成
# add_executable(running_test ${RUNNING_TEST_SOURCES})
# target_link_libraries(running_test
#   PRIVATE
#     running_impl
#     gtest_main
# )

# # テストの登録
# gtest_discover_tests(running_test)

# ## 撮影システム ##

# # テスト有効化オプション
# option(ENABLE_CAMERA_TESTS "Build camera tests" ON)

# if(ENABLE_CAMERA_TESTS)
#   find_package(OpenCV REQUIRED)

#   # 必要なインクルードディレクトリの設定 (適宜登録)
#   set(CAMERA_INCLUDE_DIRS
#     ${CAMERA_MODULE_DIR}/camera
#     ${CAMERA_MODULE_DIR}/utils
#     ${CAMERA_MODULE_DIR}/server
#     ${CAMERA_MODULE_DIR}/image_processors
#     ${PROJECT_SOURCE_DIR}/common
#     ${PROJECT_SOURCE_DIR}/common/customs
#     ${RUNNING_MODULE_DIR}/API
#     ${RUNNING_TEST_DIR}/dummy
#   )

#   # ソースファイルの登録 (適宜登録)
#   file(GLOB_RECURSE CAMERA_SOURCES
#     "${CAMERA_MODULE_DIR}/camera/*.cpp"
#     "${CAMERA_MODULE_DIR}/utils/*.cpp"
#     "${CAMERA_MODULE_DIR}/server/*.cpp"
#     "${CAMERA_MODULE_DIR}/image_processors/*.cpp"
#     "${PROJECT_SOURCE_DIR}/common/*.cpp"
#     "${RUNNING_MODULE_DIR}/API/*.cpp"
#   )

#   # ライブラリターゲットを作成
#   add_library(camera_server_impl STATIC ${CAMERA_SOURCES})
#   target_include_directories(camera_server_impl PUBLIC ${CAMERA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#   target_link_libraries(camera_server_impl ${OpenCV_LIBS})

#   # camera_server/tests ディレクトリ内の *Test.cpp をすべてテストソースとして登録
#   file(GLOB CAMERA_TEST_SOURCES
#     "${CAMERA_TEST_DIR}/*Test.cpp"
#   )

#   # テスト実行可能ファイルの作成
#   add_executable(camera_server_test ${CAMERA_TEST_SOURCES})
#   target_link_libraries(camera_server_test
#     PRIVATE
#       camera_server_impl
#       gtest_main
#       ${OpenCV_LIBS}
#   )

#   # テストの登録
#   gtest_discover_tests(camera_server_test)

# endif()
