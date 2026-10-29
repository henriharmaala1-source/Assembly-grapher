# Turn a binary file into a C++ source defining its bytes, at BUILD time.
#
#   cmake -DIN=model.onnx -DOUT=embedded.cpp -DNS=kdemo -DSYM=yoloxNano -P embed_file.cmake
#
# Emitted as string-literal pieces rather than one huge {0x.., ...} list:
# compilers take string literals in their stride where a four-million-element
# initializer costs real memory and minutes. Each piece stays under MSVC's
# 16 KB literal limit, and every byte is a \x escape followed by another
# escape, so no following character can be read into it.
file(READ "${IN}" hex HEX)
string(LENGTH "${hex}" nhex)
math(EXPR nbytes "${nhex} / 2")
set(piece 4000)                      # bytes per literal
math(EXPR pieceHex "${piece} * 2")
set(body "")
set(pos 0)
while(pos LESS nhex)
  string(SUBSTRING "${hex}" ${pos} ${pieceHex} chunk)
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "\\\\x\\1" chunk "${chunk}")
  string(APPEND body "    \"${chunk}\",\n")
  math(EXPR pos "${pos} + ${pieceHex}")
endwhile()
file(WRITE "${OUT}"
"// GENERATED from ${IN} by cmake/embed_file.cmake -- do not edit.\n"
"#include <cstddef>\n#include <cstring>\n#include <vector>\n"
"namespace ${NS} {\nnamespace {\n"
"const char* const kPieces[] = {\n${body}};\n"
"const std::size_t kSize = ${nbytes};\n"
"const std::size_t kPiece = ${piece};\n"
"}  // namespace\n"
"const unsigned char* ${SYM}Bytes() {\n"
"    static const std::vector<unsigned char> bytes = [] {\n"
"        std::vector<unsigned char> b(kSize);\n"
"        for (std::size_t i = 0, off = 0; off < kSize; ++i, off += kPiece)\n"
"            std::memcpy(b.data() + off, kPieces[i], kSize - off < kPiece ? kSize - off : kPiece);\n"
"        return b;\n"
"    }();\n"
"    return bytes.data();\n"
"}\n"
"std::size_t ${SYM}Size() { return kSize; }\n"
"}  // namespace ${NS}\n")
