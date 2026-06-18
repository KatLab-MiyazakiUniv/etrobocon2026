/**
 * @file   Course.h
 * @brief  コースの情報を保持するファイル
 * @author nishijima515
 */

#ifndef COURSE_H
#define COURSE_H

// エリア名を持つ列挙型変数（LineTrace = 0, Area2 = 1）
enum class Area { LineTrace, Area2 };

// コース名を持つ列挙型変数（Left = 0, Right = 1）
enum class Course { Left, Right };

// Edegeの状態名を持つ列挙型変数（LeftEdge = 0, RightEdge = 1）
enum class Edge { LeftEdge, RightEdge };

#endif
