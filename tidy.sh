function check_class_case() {
  echo "Checking class case"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.ClassCase, value: CamelCase} ]}" \
    2>/dev/null
}

function fix_class_case() {
  echo "Fixing class case"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.ClassCase, value: CamelCase} ]}" \
    -fix
}

function fix_function_case() {
  echo "Fixing function case"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{ CheckOptions: [ {key: readability-identifier-naming.PublicMethodCase, value: CamelCase},{key: readability-identifier-naming.ProtectedMethodCase, value: CamelCase},{key: readability-identifier-naming.PrivateMethodCase, value: CamelCase},{key: readability-identifier-naming.GlobalFunctionCase, value: CamelCase},{key: readability-identifier-naming.FunctionCase, value: CamelCase} ]}" \
    -fix
}

function fix_function_case_backup() {
  echo "Fixing function case"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.ClassMethodCase, value: CamelCase},{key: readability-identifier-naming.FunctionCase, value: CamelCase}, {key: readability-identifier-naming.MethodCase, value: CamelCase},{key: readability-identifier-naming.PublicMethodCase, value: CamelCase},{key: readability-identifier-naming.PrivateMethodCase, value: CamelCase},{key: readability-identifier-naming.ProtectedMethodCase, value: CamelCase}  ]}" \
    -fix
}

function fix_member_case() {
  echo "Fixing member case"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.MemberCase, value: lower_case}, {key: readability-identifier-naming.MemberSuffix, value: '_'}, { key: readability-identifier-naming.ConstantMemberCase, value: CamelCase }, { key: readability-identifier-naming.ConstantMemberPrefix, value: k } ]}" \
    -fix
}

function fix_variable_case() {
  echo "Fixing variable case"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.VariableCase, value: lower_case},
                              {key: readability-identifier-naming.VariableSuffix, value: ''},
                              { key: readability-identifier-naming.ConstexprVariableCase, value: CamelCase },
                              { key: readability-identifier-naming.ConstexprVariablePrefix, value: k },
                              { key: readability-identifier-naming.ConstantMemberCase, value: CamelCase },
                              { key: readability-identifier-naming.ConstantMemberPrefix, value: k },
                              { key: readability-identifier-naming.ClassConstantCase, value: CamelCase },
                              { key: readability-identifier-naming.ClassConstantPrefix, value: k }
                              ]}" \
    -fix
}

function modernize_headers() {
  echo "Modernize Headers"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-deprecated-headers' \
    -fix 1>/dev/null 2>/dev/null
}

function modernize_arrays() {
  echo "Modernize Arrays"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-avoid-c-arrays'
}

function performance() {
  echo "Performance"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,performance-unnecessary-value-param,performance-unnecessary-copy-initialization,performance-inefficient-vector-operation' \
    -fix 1>/dev/null 2>/dev/null
}

function modernize2() {
  echo "Modernize 2"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-loop-convert,modernize-avoid-bind,modernize-raw-string-literal' \
    -fix 1>/dev/null 2>/dev/null
}

function modernize3() {
  echo "Modernize 3"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-use-default,modernize-use-bool-literals,modernize-shrink-to-fit,modernize-redundant-void-arg' \
    -fix 1>/dev/null 2>/dev/null
}

# modernize-shrink-to-fit modernize-use-equals-default modernize-avoid-c-arrays  performance-  performance-inefficient-algorithm
# modernize-use-emplace modernize-use-default modernize-use-bool-literals modernize-shrink-to-fit modernize-redundant-void-arg
# modernize-loop-convert modernize-avoid-bind modernize-raw-string-literal
#modernize_headers
#check_class_case
#fix_member_case
#fix_variable_case
#performance
#modernize2
#fix_function_case
#modernize3
#fix_function_case


function bugprone_autofix() {
  echo "Bugprone autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,bugprone-argument-comment, bugprone-bool-pointer-implicit-conversion, bugprone-copy-constructor-init, bugprone-inaccurate-erase, bugprone-macro-parentheses, bugprone-misplaced-operator-in-strlen-in-alloc, bugprone-misplaced-pointer-arithmetic-in-alloc, bugprone-move-forwarding-reference, bugprone-not-null-terminated-result, bugprone-parent-virtual-call, bugprone-posix-return, bugprone-redundant-branch-condition, bugprone-reserved-identifier, bugprone-string-constructor, bugprone-string-integer-assignment, bugprone-suspicious-memset-usage, bugprone-suspicious-semicolon, bugprone-suspicious-string-compare, bugprone-swapped-arguments, bugprone-terminating-continue, bugprone-unused-raii, bugprone-virtual-near-miss' \
    -fix 
}

# bugprone with fixes
#bugprone-argument-comment, bugprone-bool-pointer-implicit-conversion, bugprone-copy-constructor-init, bugprone-inaccurate-erase, bugprone-macro-parentheses, bugprone-misplaced-operator-in-strlen-in-alloc, bugprone-misplaced-pointer-arithmetic-in-alloc, bugprone-move-forwarding-reference, bugprone-not-null-terminated-result, bugprone-parent-virtual-call, bugprone-posix-return, bugprone-redundant-branch-condition, bugprone-reserved-identifier, bugprone-string-constructor, bugprone-string-integer-assignment, bugprone-suspicious-memset-usage, bugprone-suspicious-semicolon, bugprone-suspicious-string-compare, bugprone-swapped-arguments, bugprone-terminating-continue, bugprone-unused-raii, bugprone-virtual-near-miss

function cppcore_autofix() {
  echo "Cppcore autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,cppcoreguidelines-pro-type-member-init, cppcoreguidelines-pro-type-static-cast-downcast' \
    -fix 
}

#cppcore with fixed
#cppcoreguidelines-init-variables, cppcoreguidelines-pro-bounds-constant-array-index, cppcoreguidelines-pro-type-cstyle-cast, cppcoreguidelines-pro-type-member-init, cppcoreguidelines-pro-type-static-cast-downcast

function google_autofix() {
  echo "Google checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,google-explicit-constructor, google-upgrade-googletest-case' \
    -fix 
}


#google with fixes
#google-explicit-constructor, google-upgrade-googletest-case

function misc_autofix() {
  echo "Misc checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,misc-definitions-in-headers, misc-redundant-expression, misc-static-assert, misc-uniqueptr-reset-release, misc-unused-alias-decls, misc-unused-parameters, misc-unused-using-decls, modernize-avoid-bind, modernize-concat-nested-namespaces, modernize-deprecated-headers, modernize-deprecated-ios-base-aliases' \
    -fix 
}

#misc with fixes
#misc-definitions-in-headers, misc-redundant-expression, misc-static-assert, misc-uniqueptr-reset-release, misc-unused-alias-decls, misc-unused-parameters, misc-unused-using-decls, modernize-avoid-bind, modernize-concat-nested-namespaces, modernize-deprecated-headers, modernize-deprecated-ios-base-aliases

function modernize_autofix() {
  echo "Modernize checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-loop-convert,modernize-make-shared,modernize-make-unique,modernize-pass-by-value,modernize-raw-string-literal,modernize-redundant-void-arg,modernize-replace-auto-ptr,modernize-replace-disallow-copy-and-assign-macro,modernize-replace-random-shuffle,modernize-return-braced-init-list,modernize-shrink-to-fit,modernize-unary-static-assert,modernize-use-auto,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-emplace,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-noexcept,modernize-use-nullptr,modernize-use-override,modernize-use-transparent-functors,modernize-use-uncaught-exceptions,modernize-use-using' \
    -fix 
}

#modernize with fixes
#removed modernize-use-trailing-return-type modernize-use-nodiscard
#modernize-loop-convert,modernize-make-shared,modernize-make-unique,modernize-pass-by-value,modernize-raw-string-literal,modernize-redundant-void-arg,modernize-replace-auto-ptr,modernize-replace-disallow-copy-and-assign-macro,modernize-replace-random-shuffle,modernize-return-braced-init-list,modernize-shrink-to-fit,modernize-unary-static-assert,modernize-use-auto,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-emplace,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-nodiscard,modernize-use-noexcept,modernize-use-nullptr,modernize-use-override,modernize-use-trailing-return-type,modernize-use-transparent-functors,modernize-use-uncaught-exceptions,modernize-use-using

function performance_autofix() {
  echo "Perf checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,performance-faster-string-find,performance-for-range-copy,performance-inefficient-algorithm,performance-inefficient-vector-operation,performance-trivially-destructible,performance-type-promotion-in-math-fn,performance-unnecessary-value-param' \
    -fix 
}

#performance with fixes
#removed erformance-move-const-arg,performance-move-constructor-init,performance-noexcept-move-constructor
#performance-faster-string-find,performance-for-range-copy,performance-inefficient-algorithm,performance-inefficient-vector-operation,performance-move-const-arg,performance-move-constructor-init,performance-noexcept-move-constructor,performance-trivially-destructible,performance-type-promotion-in-math-fn,performance-unnecessary-value-param

function readability_autofix() {
  echo "Readability checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-braces-around-statements, readability-const-return-type, readability-container-size-empty, readability-implicit-bool-conversion, readability-inconsistent-declaration-parameter-name, readability-isolate-declaration, readability-make-member-function-const, readability-misplaced-array-index, readability-named-parameter, readability-non-const-parameter, readability-qualified-auto, readability-redundant-access-specifiers, readability-redundant-control-flow, readability-redundant-declaration, readability-redundant-function-ptr-dereference,readability-static-accessed-through-instance
' \
    -fix 
}

#readability with fixes
#readability-braces-around-statements, readability-const-return-type, readability-container-size-empty, readability-implicit-bool-conversion, readability-inconsistent-declaration-parameter-name, readability-isolate-declaration, readability-make-member-function-const, readability-misplaced-array-index, readability-named-parameter, readability-non-const-parameter, readability-qualified-auto, readability-redundant-access-specifiers, readability-redundant-control-flow, readability-redundant-declaration, readability-redundant-function-ptr-dereference,readability-static-accessed-through-instance


#cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DDEBUG=true ..

#fix_member_case
#fix_class_case
#fix_variable_case

#fix_function_case

#modernize_headers
#performance
#modernize2

#check_class_case
#fix_class_case
#fix_variable_case
#fix_member_case
#fix_function_case

#passing
#bugprone_autofix
#google_autofix
#misc_autofix (see warnings)
#readability_autofix
#performance_autofix (move / sqrt function)
modernize_autofix
