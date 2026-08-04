/**
 * @file   Decrypter.cpp
 * @brief  暗号化文字列を復号するクラス
 * @author yutaro-1214 HaruArima08
 */

#include "Decrypter.h"

std::string Decrypter::decrypt(const std::string& key, const std::string& encryptedText)
{
  auto decoded = decodeBase64(encryptedText);
  if(!decoded) {
    Logger::error("Decrypter: Base64でコードに失敗しました。");
    return "";
  }

  // "Salted__" ヘッダとSaltを確認（openssl enc形式）
  if(std::memcmp(decoded->data(), "Salted__", 8) != 0) {
    Logger::error("Decrypter: Saltヘッダが見つかりません。");
    return "";
  }
  const unsigned char* salt = decoded->data() + 8;

  auto aesKey = deriveKey(key, salt);
  if(!aesKey) return "";

  return decryptAES(decoded->data() + 16, static_cast<int>(decoded->size()) - 16, aesKey->data());
}

std::optional<std::vector<unsigned char>> Decrypter::decodeBase64(const std::string& encoded)
{
  std::vector<unsigned char> decoded(encoded.size());
  int length = EVP_DecodeBlock(
      decoded.data(), reinterpret_cast<const unsigned char*>(encoded.c_str()), encoded.length());
  if(length < 16) {
    Logger::error("Decrypter: Base64デコードに失敗しました。");
    return std::nullopt;
  }

  // '='パディング分を差し引いて実際のバイト数を算出
  for(auto it = encoded.rbegin(); it != encoded.rend() && *it == '='; ++it) length--;
  decoded.resize(length);
  return decoded;
}

std::optional<std::array<unsigned char, 16>> Decrypter::deriveKey(const std::string& key,
                                                                  const unsigned char* salt)
{
  std::array<unsigned char, 16> aesKey;
  if(!PKCS5_PBKDF2_HMAC(key.c_str(), static_cast<int>(key.length()), salt, 8, 10000, EVP_sha256(),
                        static_cast<int>(aesKey.size()), aesKey.data())) {
    Logger::error("Decrypter: 鍵導出に失敗しました。");
    return std::nullopt;
  }
  return aesKey;
}

std::string Decrypter::decryptAES(const unsigned char* cipherText, int cipherLength,
                                  const unsigned char* aesKey)
{
  EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
  if(ctx == nullptr) {
    Logger::error("Decrypter: 暗号コンテキストの生成に失敗しました。");
    return "";
  }

  if(!EVP_DecryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, aesKey, nullptr)) {
    EVP_CIPHER_CTX_free(ctx);
    Logger::error("Decrypter: 復号の初期化に失敗しました。");
    return "";
  }

  std::vector<unsigned char> plain(cipherLength + EVP_MAX_BLOCK_LENGTH);
  int outLen1 = 0, outLen2 = 0;

  if(!EVP_DecryptUpdate(ctx, plain.data(), &outLen1, cipherText, cipherLength)) {
    EVP_CIPHER_CTX_free(ctx);
    Logger::error("Decrypter: 復号処理に失敗しました。");
    return "";
  }

  if(!EVP_DecryptFinal_ex(ctx, plain.data() + outLen1, &outLen2)) {
    EVP_CIPHER_CTX_free(ctx);
    Logger::error("Decrypter: 復号の終了処理に失敗しました。");
    return "";
  }

  EVP_CIPHER_CTX_free(ctx);
  return std::string(reinterpret_cast<char*>(plain.data()), outLen1 + outLen2);
}
