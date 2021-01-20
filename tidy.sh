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

#cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DDEBUG=true ..

#fix_member_case
#fix_class_case
#fix_variable_case

#fix_function_case

#modernize_headers
#performance
#modernize2

#check_class_case
fix_class_case
fix_variable_case
fix_member_case
#fix_function_case


