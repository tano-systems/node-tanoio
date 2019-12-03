{
  'variables': {
    'use_system_libtanoio%': 'true',
  },
  'targets': [
    {
      'target_name': 'node_tanoio',
      'sources': [
        'src/tanoio.cpp',
      ],
      'include_dirs': [
        "<!(node -e \"require('nan')\")",
      ],
      'conditions': [
        ['OS=="linux" and use_system_libtanoio=="true"', {
          'include_dirs+': [
            '<!@(pkg-config libtanoio --cflags | sed s/-I//g || echo "")',
          ],
          'libraries': [
            '<!@(pkg-config libtanoio --libs || echo "")',
          ],
        }],
        ['OS=="mac" and use_system_libtanoio=="true"', {
          'include_dirs+': [
            '<!@(pkg-config libtanoio --cflags | sed s/-I//g || echo "")',
          ],
          'libraries': [
            '<!@(pkg-config libtanoio --libs || echo "")',
          ],
        }],
      ],
    },
  ],
}
