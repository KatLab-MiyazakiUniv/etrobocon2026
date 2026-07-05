/**
 * @file Button.h
 * @brief ボタンクラス(ダミー)
 * @author okuyama0528 sadomiya-sousi
 */

#ifndef BUTTON_H
#define BUTTON_H

namespace spikeapi {
  class Button {
   public:
    /**
     * コンストラクタ
     */
    Button(void) = default;

    /**
     * LEFTボタンが押されているかをランダムに判定
     */
    bool isLeftPressed() const { return std::rand() % 2 == 0; }

    /**
     * LEFTボタンが押されているかをランダムに判定
     */
    bool isCenterPressed() const { return std::rand() % 2 == 0; }

    /**
     * RIGHTボタンが押されているかをランダムに判定
     */
    bool isRightPressed() const { return std::rand() % 2 == 0; }

    /**
     * インスタンス生成が正常にできたかどうかを確認するための共通メソッド
     * Buttonでは複数生成が問題ないので、常にfalseを返す
     */
    bool hasError();
  };
}  // namespace spikeapi

#endif