
function readability_autofix() {
  echo "Checking code readability standards"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-deprecated-headers,readability-identifier-naming, readability-braces-around-statements, readability-const-return-type, readability-container-size-empty, readability-implicit-bool-conversion, readability-inconsistent-declaration-parameter-name, readability-isolate-declaration, readability-make-member-function-const, readability-misplaced-array-index, readability-named-parameter, readability-non-const-parameter, readability-qualified-auto, readability-redundant-access-specifiers, readability-redundant-control-flow, readability-redundant-declaration, readability-redundant-function-ptr-dereference,readability-static-accessed-through-instance,readability-redundant-member-init' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.ClassCase, value: CamelCase},
                              {key: readability-identifier-naming.ClassMethodCase, value: CamelCase},
                              {key: readability-identifier-naming.GlobalFunctionCase, value: CamelCase},
                              {key: readability-identifier-naming.FunctionCase, value: CamelCase},
                              {key: readability-identifier-naming.EnumConstantCase, value: CamelCase},
                              {key: readability-identifier-naming.EnumConstantPrefix, value: k},
                    	       {key: readability-identifier-naming.GlobalConstantCase, value: CamelCase},
                              {key: readability-identifier-naming.GlobalConstantPrefix, value: k},
                  	       {key: readability-identifier-naming.StaticConstantCase, value: CamelCase},
                              {key: readability-identifier-naming.StaticConstantPrefix, value: k},
                	       {key: readability-identifier-naming.ScopedEnumConstantCase, value: CamelCase},
                              {key: readability-identifier-naming.ScopedEnumConstantPrefix, value: k},
                              {key: readability-identifier-naming.ConstexprVariableCase, value: CamelCase },
                              {key: readability-identifier-naming.ConstexprVariablePrefix, value: k },
                              {key: readability-identifier-naming.ClassConstantCase, value: CamelCase },
                              {key: readability-identifier-naming.ClassConstantPrefix, value: k },
                              {key: readability-identifier-naming.MemberCase, value: lower_case}, 
                              {key: readability-identifier-naming.MemberSuffix, value: '_'}, 
                              {key: readability-identifier-naming.VariableCase, value: lower_case},
                              {key: readability-identifier-naming.VariableSuffix, value: ''},
                              {key: readability-identifier-naming.ConstantPointerParameterCase, value: lower_case},
                              {key: readability-identifier-naming.LocalConstantPointerCase, value: lower_case}
                               ]}" \
    -fix
}

#readability with fixes
#readability-braces-around-statements, readability-const-return-type, readability-container-size-empty, readability-implicit-bool-conversion, readability-inconsistent-declaration-parameter-name, readability-isolate-declaration, readability-make-member-function-const, readability-misplaced-array-index, readability-named-parameter, readability-non-const-parameter, readability-qualified-auto, readability-redundant-access-specifiers, readability-redundant-control-flow, readability-redundant-declaration, readability-redundant-function-ptr-dereference,readability-static-accessed-through-instance,readability-redundant-member-init

function keychecks_autofix() {
  echo "Key Checks autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,bugprone-argument-comment, bugprone-bool-pointer-implicit-conversion, bugprone-copy-constructor-init, bugprone-inaccurate-erase, bugprone-macro-parentheses, bugprone-misplaced-operator-in-strlen-in-alloc, bugprone-misplaced-pointer-arithmetic-in-alloc, bugprone-move-forwarding-reference, bugprone-not-null-terminated-result, bugprone-parent-virtual-call, bugprone-posix-return, bugprone-redundant-branch-condition, bugprone-reserved-identifier, bugprone-string-constructor, bugprone-string-integer-assignment, bugprone-suspicious-memset-usage, bugprone-suspicious-semicolon, bugprone-suspicious-string-compare, bugprone-swapped-arguments, bugprone-terminating-continue, bugprone-unused-raii, bugprone-virtual-near-miss, cppcoreguidelines-pro-type-static-cast-downcast, google-explicit-constructor, google-upgrade-googletest-case, misc-definitions-in-headers, misc-redundant-expression, misc-static-assert, misc-uniqueptr-reset-release, misc-unused-alias-decls, misc-unused-parameters, misc-unused-using-decls' \
    -fix 
}

function check_bugprone() {
  echo "Checking Bugprone behaviors"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,bugprone-*' 2>/dev/null
}

#bugprone-signal-handler

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
    -checks='-*,cppcoreguidelines-pro-type-static-cast-downcast' \
    -fix 
}

#Removed cppcoreguidelines-pro-type-member-init,

#cppcore with fixed (cppcoreguidelines-pro-type-member-init)
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
    -checks='-*,misc-definitions-in-headers, misc-redundant-expression, misc-static-assert, misc-uniqueptr-reset-release, misc-unused-alias-decls, misc-unused-parameters, misc-unused-using-decls' \
    -fix 
}

#misc with fixes
#misc-definitions-in-headers, misc-redundant-expression, misc-static-assert, misc-uniqueptr-reset-release, misc-unused-alias-decls, misc-unused-parameters, misc-unused-using-decls

function modernize_autofix() {
  echo "Modernize checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,modernize-avoid-bind, modernize-concat-nested-namespaces, modernize-deprecated-headers, modernize-deprecated-ios-base-aliases,modernize-loop-convert,modernize-make-shared,modernize-make-unique,modernize-pass-by-value,modernize-raw-string-literal,modernize-redundant-void-arg,modernize-replace-auto-ptr,modernize-replace-disallow-copy-and-assign-macro,modernize-replace-random-shuffle,modernize-return-braced-init-list,modernize-shrink-to-fit,modernize-unary-static-assert,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-emplace,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-noexcept,modernize-use-nullptr,modernize-use-override,modernize-use-transparent-functors,modernize-use-uncaught-exceptions,modernize-use-using' \
    -fix 
}

#modernize with fixes
#removed modernize-use-trailing-return-type modernize-use-nodiscard  modernize-use-auto
#modernize-avoid-bind, modernize-concat-nested-namespaces, modernize-deprecated-headers, modernize-deprecated-ios-base-aliases,modernize-loop-convert,modernize-make-shared,modernize-make-unique,modernize-pass-by-value,modernize-raw-string-literal,modernize-redundant-void-arg,modernize-replace-auto-ptr,modernize-replace-disallow-copy-and-assign-macro,modernize-replace-random-shuffle,modernize-return-braced-init-list,modernize-shrink-to-fit,modernize-unary-static-assert,modernize-use-auto,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-emplace,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-nodiscard,modernize-use-noexcept,modernize-use-nullptr,modernize-use-override,modernize-use-trailing-return-type,modernize-use-transparent-functors,modernize-use-uncaught-exceptions,modernize-use-using

function performance_autofix() {
  echo "Perf checks with autofix"
  run-clang-tidy-11.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,performance-faster-string-find,performance-for-range-copy,performance-inefficient-algorithm,performance-inefficient-vector-operation,performance-trivially-destructible,performance-type-promotion-in-math-fn,performance-unnecessary-value-param,performance-move-const-arg,performance-move-constructor-init,performance-noexcept-move-constructor' \
    -fix 
}

#performance with fixes
#removed performance-move-const-arg,performance-move-constructor-init,performance-noexcept-move-constructor
#performance-faster-string-find,performance-for-range-copy,performance-inefficient-algorithm,performance-inefficient-vector-operation,performance-move-const-arg,performance-move-constructor-init,performance-noexcept-move-constructor,performance-trivially-destructible,performance-type-promotion-in-math-fn,performance-unnecessary-value-param


#cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DDEBUG=true ..


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
#modernize_autofix

#readability_autofix
#result_naming=$?

keychecks_autofix
modernize_autofix
performance_autofix


echo "Result: " $result_naming
#echo "Result: " $result_bugprone
#echo "Result: " $result_modernize
#echo "Result: " $result_misc
#echo "Result: " $result_cppcore

