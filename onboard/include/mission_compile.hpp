#pragma once
// ---------------------------------------------------------------------------
// THE MISSION COMPILER: .kms text -> kms::Program. Runs where missions are
// WRITTEN (kestrel mission compile, the GUI, the tests) -- it is not linked
// into the Pi's binary at all. The language is documented with examples in
// onboard/docs/mission-scripts.md.
//
// It is the place where mistakes are cheap, so it is strict:
//   * every name resolved -- an unknown place or a misspelt keyword is an
//     error with its line and column, not a surprise at 20 m
//   * units checked and converted -- `climb 5 s` is an error, `2 min` is 120 s
//   * everything that waits has a bound: loops are `repeat N` or `while ...`
//     with a timeout, every motion has a timeout (defaults stated in the doc)
//   * places checked against the fence where they can be: a target 60 m from
//     the start in a 40 m fence fails here, not in the air
//   * patterns translated into plain instructions: orbit and survey become a
//     list of gotos the Pi just follows
// ---------------------------------------------------------------------------

#include <string>
#include <vector>

#include "mission_program.hpp"

namespace kms {

struct Diagnostic {
    int line = 0, col = 0;
    bool error = true;           // false: a warning
    std::string message;
};

struct CompileResult {
    bool ok = false;
    Program program;
    std::vector<Diagnostic> diags;
    // "file:line:col: error: message", one per line -- what the CLI prints.
    std::string report(const std::string& file) const;
};

CompileResult compile(const std::string& source, const std::string& sourceName);
CompileResult compileFile(const std::string& path);

}  // namespace kms
