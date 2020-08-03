function fix_class_case() {
  echo "Fixing class case"
  for i in 1 2; do
    run-clang-tidy-10.py -header-filter="(src|test|microbench|simulator|data).*" \
      -checks='-*,readability-identifier-naming' \
      -config="{CheckOptions: [ {key: readability-identifier-naming.ClassCase, value: CamelCase} ]}" \
      -fix 1>/dev/null 2>/dev/null
  done
}

function fix_member_case() {
  echo "Fixing member case"
  run-clang-tidy-10.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.MemberCase, value: lower_case}, {key: readability-identifier-naming.MemberSuffix, value: '_'} ]}" \
    -fix 1>/dev/null 2>/dev/null
}

function fix_variable_case() {
  echo "Fixing variable case"
  run-clang-tidy-10.py -header-filter="(src|test|microbench|simulator|data).*" \
    -checks='-*,readability-identifier-naming' \
    -config="{CheckOptions: [ {key: readability-identifier-naming.VariableCase, value: lower_case},
                              {key: readability-identifier-naming.VariableSuffix, value: ''},
                              { key: readability-identifier-naming.ConstexprVariableCase, value: CamelCase },
                              { key: readability-identifier-naming.ConstexprVariablePrefix, value: k }
                              ]}" \
    -fix 1>/dev/null 2>/dev/null
}

fix_class_case
fix_member_case
fix_variable_case

