from jinja2 import Template
from pathlib import Path

_plot_width = {
  'benchmark_app_evaluation.svg': 1200,
  'cpu_ram_plot.svg': 1000
}

_results_index_template = Template('''
<!DOCTYPE html>
<html>
<head>
</head>
<body>

<h2>Results for benchmark run {{result_id}}</h2>

{% for benchmark in benchmarks %}
  <h3>{{benchmark}}</h3>

  <div align="left">
    <a href="{{benchmark}}/log.tgz">log.tgz</a>
  </div>

  {% for plot in plots %}
  <div align="left">
    <img src="{{benchmark}}/plots/{{plot}}" alt="{{plot}}" width="{{plot_width[plot]}}">
  </div>
  {% endfor %}
  
{% endfor %}

</body>
</html>
''')

if __name__ == '__main__':
    import sys
    root_path = Path(sys.argv[1])
    benchmarks_paths = [ p for p in root_path.iterdir() if p.is_dir() ]
    index_path = root_path / 'index.html'
    with open(str(index_path), 'w') as out_f:
        rendered_index = _results_index_template.render(
          result_id=root_path.name,
          benchmarks=[ p.name for p in benchmarks_paths ],
          plots=[ p.name for p in (benchmarks_paths[0] / 'plots').iterdir() if p.is_file() ],
          plot_width=_plot_width
        )
        out_f.write(rendered_index)
