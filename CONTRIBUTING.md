# Contributing 


## Before creating a pull request

  * Ensure that the code compiles without warnings and all tests pass.

  * Format the source code using `clang-format`. Running `clang-format -i *.cc
    *.h` will format source and header files in the current directory to match
    Agora's code style. There are also editor plugins for `clang-format`
    ([example](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)). We recommend using clang-format version 11 or above.

## Code style

  * Currently the code is not fully consistent with these style guidelines. It
    is a work in progress.

  * At a high level, we aim to follow Google's C++ style
    [guide](https://google.github.io/styleguide/cppguide.html).

  * [Naming](https://google.github.io/styleguide/cppguide.html#Naming):
    * `lower_camel_case` for variable names.
    * `UpperCamelCase` for class, struct, and function names.
    * `kConstant` for static storage duration constants. Otherwise the usual variable nameing rules apply.
    * Source file names should end in `.cc`, header file names in `.h`.

  * Classes:
    * Mark eligible class functions and members as `const`, `private`, or
      `static`.
    * Eligible classes should have corresponding tests.

  * Avoid macros unless absolutely
    [necessary](https://google.github.io/styleguide/cppguide.html#Preprocessor_Macros).
    For example, macros are acceptable to disable compilation of files that
    depend on proprietary libraries. So instead of:

    <pre>
    #ifdef USE_LDPC
        std::string raw_data_filename = x;
    #else
        std::string raw_data_filename = y;
    #endif
    </pre>

    prefer:
    <pre>
    std::string raw_data_filename = kUseLDPC ? x : y;
    </pre>

  * Avoid magic numbers. Instead of:
    <pre>
    n_rows = (ldpc_config.bg == 1) ? 46 : 42;
    </pre>

    prefer:
    <pre>
    n_rows = (ldpc_config.bg == 1) ? kBg1RowTotal : kBg2RowTotal;
    </pre>

  * Avoid variable copies. Instead of `size_t fft_thread_num =
    cfg->fft_thread_num;`, prefer using `cfg->fft_thread_num` directly.

  * Use enum classes instead of enums or macros. So instead of:

    <pre>
    #define PRINT_RX_PILOTS 0
    #define PRINT_RX 1
    </pre>
 
    prefer:
    <pre>
    enum class PrintType {kRXPilots, kRX};
    </pre>

  * Add newlines between distinct blocks of code. See the vertical whitespace
    [section](https://google.github.io/styleguide/cppguide.html#Vertical_Whitespace).

  * Comments
    ([link](https://google.github.io/styleguide/cppguide.html#Comments)):
    * Use proper grammar and capitalization in comment text.
    * Comments should fit in 80 columns.
    * Add comments for non-obvious blocks of code.
    * Avoid commented-out blocks of code. For example, if a block of code is
      needed for debugging, wrap it inside a boolean flag (e.g., `kVerbose`).

  * Use `size_t` as the integer type unless negative integers are needed.

  * Mark eligible function arguments and implementation variables as `const`.

  * Use C++-style casts with `static_cast` and `reinterpret_cast` instead of
    C-style casts.

  * Use auto type deduction when the type is clear. So instead of:
    <pre>
    struct Packet *pkt = new struct Packet[kNumPackets];
    </pre>

    prefer:
    <pre>
    auto *pkt = new Packet[kNumPackets];
    </pre>

## Documentation requirements
  * Each file must have a comment at the top explaining what the file's purpose.
  * Each class must have a top-level comment explaining the class's purpose.
  * Non-trivial class members and functions should have documentation comments
    in the class header file.
