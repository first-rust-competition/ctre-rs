import re

RUST_TYPES = {
    'short': 'i16',
    'int': 'i32',
    'uint32_t': 'u32',
    'double': 'f64',
    'float': 'f32',
}


def to_snake_case(s: str) -> str:
    return re.sub(r'(?<=[a-z0-9])[A-Z]|(?!^)[A-Z](?=[a-z])', r'_\g<0>', s).lower()


def function_hook(fn, data):
    m = re.match(r'c_MotController_(.*)', fn['name'])
    assert m, f"unexpected function {fn['name']}"

    snake_name = to_snake_case(m.group(1))
    fn['meth_name'] = snake_name
    if snake_name == 'create1':
        fn['meth_name'] = 'new'
        fn['rust_returns'] = 'Self'
        return

    in_params = []
    out_params = []

    for i, p in enumerate(fn['parameters'][1:]):
        p['snake_name'] = to_snake_case(p['name'] or f'param{i}')
        p['rust_type'] = RUST_TYPES.get(p['raw_type'], p['raw_type'])
        p['rust_decl'] = f"{p['snake_name']}: {p['rust_type']}"

        if p['pointer']:
            out_params.append(p)
        else:
            in_params.append(p)

    has_error_code = fn['returns'] == 'ctre::phoenix::ErrorCode'
    rust_return_types = [p['rust_type'] for p in out_params]
    if not rust_return_types:
        fn['rust_returns'] = 'ErrorCode' if has_error_code else None
    elif len(rust_return_types) == 1:
        fn['rust_returns'] = 'Result<%s>' % rust_return_types[0]
        fn['simple_getter'] = True
    else:
        fn['rust_returns'] = 'Result<(%s)>' % ', '.join(rust_return_types)

    fn['in_params'] = in_params
    fn['out_params'] = out_params
