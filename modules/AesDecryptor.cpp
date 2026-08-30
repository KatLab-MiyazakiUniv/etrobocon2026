/**
 * @file   AesDecryptor.cpp
 * @brief  AES-128(ECB)による復号処理クラス
 * @author migaku2645
 */

#include "AesDecryptor.h"

AesDecryptor::AesDecryptor(const std::string& _key, const std::string& _ciphertext)
  : key(_key), ciphertext(_ciphertext)
{
  if(!validateKey()) {
    Logger::error("AesDecryptor: 復号キーは4桁の数字で指定してください。");
  }

  LOG_CREATE("AesDecryptor");
}

AesDecryptor::~AesDecryptor()
{
  LOG_DESTROY("AesDecryptor");
}

bool AesDecryptor::validateKey()
{
  if(key.size() != 4) {
    return false;
  }

  for(char c : key) {
    if(c < '0' || c > '9') {
      return false;
    }
  }

  return true;
}

void AesDecryptor::createKey(unsigned char aesKey[16])
{
  for(int i = 0; i < 4; i++) {
    // 各桁を数字に変換
    int digit = key[i] - '0';

    // int型4バイトとして格納
    std::memcpy(aesKey + i * 4, &digit, sizeof(int));
  }
}

std::vector<unsigned char> AesDecryptor::decodeBase64(const std::string& input)
{
  BIO* bio = BIO_new_mem_buf(input.data(), static_cast<int>(input.size()));

  if(bio == nullptr) {
    Logger::error("AesDecryptor: Base64デコーダの作成に失敗しました。");
    return {};
  }

  BIO* b64 = BIO_new(BIO_f_base64());

  if(b64 == nullptr) {
    BIO_free(bio);

    Logger::error("AesDecryptor: Base64デコーダの初期化に失敗しました。");
    return {};
  }

  // Base64の改行を無視
  BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);

  bio = BIO_push(b64, bio);

  std::vector<unsigned char> output(input.size());

  int length = BIO_read(bio, output.data(), static_cast<int>(output.size()));

  BIO_free_all(bio);

  if(length <= 0) {
    Logger::error("AesDecryptor: Base64のデコードに失敗しました。");
    return {};
  }

  output.resize(length);

  return output;
}

std::string AesDecryptor::decrypt()
{
  // 復号キーを確認
  if(!validateKey()) {
    Logger::error("AesDecryptor: 復号キーが不正です。");
    return "";
  }

  // 4桁キーを16バイトのAES鍵に変換
  unsigned char aesKey[16];

  createKey(aesKey);

  // Base64をデコード
  std::vector<unsigned char> encryptedData = decodeBase64(ciphertext);

  if(encryptedData.empty()) {
    Logger::error("AesDecryptor: 暗号文のデコードに失敗しました。");
    return "";
  }

  // AESコンテキストを作成
  EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();

  if(ctx == nullptr) {
    Logger::error("AesDecryptor: AES復号コンテキストの作成に失敗しました。");
    return "";
  }

  // AES-128-ECBを初期化
  if(EVP_DecryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, aesKey, nullptr) != 1) {
    Logger::error("AesDecryptor: AES-128-ECBの初期化に失敗しました。");

    EVP_CIPHER_CTX_free(ctx);

    return "";
  }

  // 復号結果を格納するバッファ
  std::vector<unsigned char> plaintext(encryptedData.size() + EVP_MAX_BLOCK_LENGTH);

  int plaintextLength = 0;
  int finalLength = 0;

  // AES復号
  if(EVP_DecryptUpdate(ctx, plaintext.data(), &plaintextLength, encryptedData.data(),
                       static_cast<int>(encryptedData.size()))
     != 1) {
    Logger::error("AesDecryptor: AES復号に失敗しました。");

    EVP_CIPHER_CTX_free(ctx);

    return "";
  }

  // パディングを処理
  if(EVP_DecryptFinal_ex(ctx, plaintext.data() + plaintextLength, &finalLength) != 1) {
    Logger::error("AesDecryptor: AES復号の最終処理に失敗しました。");

    EVP_CIPHER_CTX_free(ctx);

    return "";
  }

  EVP_CIPHER_CTX_free(ctx);

  plaintext.resize(plaintextLength + finalLength);

  Logger::info("AesDecryptor: AES復号が完了しました。");

  return std::string(plaintext.begin(), plaintext.end());
}