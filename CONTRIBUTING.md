# Contributing 


## Before creating a pull request

  * Ensure that the code compiles without warnings and all tests pass.

  * Format the source code using `clang-format`. Running `clang-format -i *.cc
    *.h` will format source and header files in the current directory to match
    Millipede's code style. There are also
    [plugins](https://github.com/google/vim_codefmt).


## Code style

  * Currently the code is not fully consistent with these style guidelines. It
    is a work in progress.

  * At a high level, we aim to follow Google's C++ style
    [guide](https://google.github.io/styleguide/cppguide.html).

  * [Naming](https://google.github.io/styleguide/cppguide.html#Naming):
    * `lower_camel_case` for variable names.
    * `UpperCamelCase` for class, struct, and function names.
    * `kConstant` for constants.
    * Source file names should end in `.cc`, header file names in `.h`.

  * Classes:
    * Class members and functions should have documentation comments in the
      class header file.
    * Mark eligible class functions and members as `const`, `private`, or
      `static`.
    * Eligible classes should have corresponding tests.

  * Avoid macros unless necessary. For example, macros are acceptable to disable
    compilation of files that depend on proprietary libraries. So instead of:

    ```
    #ifdef USE_LDPC
        std::string raw_data_filename = x;
    #else
        std::string raw_data_filename = y;
    #endif
    ```

    prefer:
    ```
    std::string raw_data_filename = kUseLDPC ? x : y;
    ```

  * Avoid magic numbers. Instead of:
    ```
    n_rows = (ldpc_config.bg == 1) ? 46 : 42;
    ```

    prefer:
    ```
    n_rows = (ldpc_config.bg == 1) ? kBg1RowTotal : kBg2RowTotal;
    ```

  * Avoid variable copies. Instead of `size_t fft_thread_num =
    cfg->fft_thread_num);`, prefer using `cfg->fft_thread_num` directly.

  * Use enum classes instead of enums or macros. So instead of:

    ```
    #define PRINT_RX_PILOTS 0
    #define PRINT_RX 1
    ```
 
    prefer:
		```
		enum class PrintType {kRXPilots, kRX};
		```

  * Add newlines between distinct blocks of code. See the vertical whitespace
    [section](https://google.github.io/styleguide/cppguide.html#Vertical_Whitespace).

  * Comments
    ([link](https://google.github.io/styleguide/cppguide.html#Comments)):
    * Use proper grammar.
    * Add comments for non-obvious blocks of code.
    * Avoid commented-out block of code in committed files. For example, if the
      block of code is needed for debugging, wrap it inside a boolean flag
      (e.g., `kVerbose`).

  * Use `size_t` as the integer type unless negative integers are needed.

  * Mark elgible function arguments and implementation variables as `const`

  * Use C++-style casts with `static_cast` and `reinterpret_cast` instead of
    C-style casts.
