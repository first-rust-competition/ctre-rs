import re

RUST_TYPES = {
    'short': 'i16',
    'int': 'i32',
    'uint32_t': 'u32',
    'double': 'f64',
    'float': 'f32',
}


def to_snake_case(s: str) -> str:
    return re.sub(r'(?<=[a-z0-9])[A-Z]|(?!^|_)[A-Z](?=[a-z])', r'_\g<0>', s).lower()


def function_hook(fn, data):
    m = re.match(r'c_%s_(.*)' % data['mod_name'], fn['name'])
    assert m, f"unexpected function {fn['name']}"

    snake_name = to_snake_case(m.group(1))

    handle_idx = -1
    in_params = []
    out_params = []

    for i, p in enumerate(fn['parameters']):
        p['snake_name'] = to_snake_case(p['name'] or f'param{i}')
        rust_type = RUST_TYPES.get(p['raw_type'], p['raw_type'])
        if p['array']:
            if 'array_size' in p:
                rust_type = f"[{rust_type}; {p['array_size']}]"
            else:
                # dummy
                rust_type = f"[{rust_type}]"

        p['rust_type'] = qual_rust_type = rust_type
        p['rust_decl'] = f"{p['snake_name']}: {p['rust_type']}"

        if p['name'] == 'handle':
            handle_idx = i
        elif p['pointer'] or p['array']:  # assume arrays are out params (they are so far)
            qual_rust_type = '&mut ' + rust_type
            out_params.append(p)
        else:
            in_params.append(p)
        p['qual_rust_type'] = qual_rust_type

    has_error_code = fn['returns'] == 'ctre::phoenix::ErrorCode'
    rust_return_types = [p['rust_type'] for p in out_params]
    if not rust_return_types:
        fn['rust_returns'] = 'ErrorCode' if has_error_code else None
    elif len(rust_return_types) == 1:
        fn['rust_returns'] = 'Result<%s>' % rust_return_types[0]
        fn['simple_getter'] = True
        if snake_name.startswith('get_'):
            snake_name = snake_name[4:]
    else:
        fn['rust_returns'] = 'Result<(%s)>' % ', '.join(rust_return_types)

    fn['meth_name'] = snake_name
    fn['handle_param_idx'] = handle_idx
    fn['in_params'] = in_params
    fn['out_params'] = out_params
    fn['num_array_params'] = sum(p['array'] for p in out_params)

    if snake_name == 'create1':
        fn['meth_name'] = 'new'
        fn['rust_returns'] = 'Self'


def class_hook(cls, data):
    for p in cls['properties']['public']:
        p['snake_name'] = to_snake_case(p['name'])
