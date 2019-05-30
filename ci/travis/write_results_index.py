from jinja2 import Template
from pathlib import Path

_results_index_template = Template('''
<!DOCTYPE html>
<html>
<head>
</head>
<body>

<h2>Results</h2>

<ul>
  {% for result_file in result_files %}
  <li>
    <a href="{{result_file.name}}">{{result_file.name}}</a>
  </li>
  {% endfor %}
</ul> 

</body>
</html>
''')

if __name__ == '__main__':
    import sys
    results_root = sys.argv[1]
    index_path = Path(results_root) / 'index.html'
    with open(str(index_path), 'w') as out_f:
        rendered_index = _results_index_template.render(result_files=Path(results_root).glob('*.csv'))
        out_f.write(rendered_index)
