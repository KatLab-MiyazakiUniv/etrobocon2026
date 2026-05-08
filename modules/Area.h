/**
 * @file   Area.h
 * @brief  エリア名を保持するファイル
 * @author nishijima515
 */


// AreaMasterにAreaを定義するとMotionParserで使えなかった
// AreaMasterとMotionParserで共有できるようにする
#ifndef AREA_H
#define AREA_H

// エリア名を持つ列挙型変数（LineTrace = 0, Area2 = 1）
enum Area { LineTrace, Area2 };

#endif
