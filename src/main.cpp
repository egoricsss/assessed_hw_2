#include <algorithm>
#include <cstddef>
#include <iostream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Константа для обозначения конца списка рёбер при выводе
constexpr std::string kEndOfEdgesMarker = "   -1";

/**
 * Класс для представления ориентированного графа и работы с ним.
 * Поддерживает операции поиска сильно связных компонент (SCC),
 * построения обратного графа и метаграфа.
 */
class Graph {
 private:
  size_t current_scc_count_;                    // Текущее количество SCC
  std::vector<std::vector<size_t>> adjacency_;  // Список смежности
  std::stack<size_t> processing_order_;         // Порядок обработки вершин
  std::vector<size_t> vertex_component_ids_;    // ID компонент для вершин

  /// Очищает стек порядка обработки
  void ClearProcessingStack() { processing_order_ = std::stack<size_t>(); }

  /**
   * Рекурсивный обход в глубину с обработкой перед и после посещения вершины.
   * @param vertex Текущая вершина
   * @param visited Массив посещённых вершин
   * @param pre_visit Действие перед посещением вершины
   * @param post_visit Действие после посещения вершины
   */
  template <typename PreVisit, typename PostVisit>
  void DepthFirstVisit(size_t vertex, std::vector<bool>& visited, PreVisit&& pre_visit, PostVisit&& post_visit) {
    visited[vertex] = true;
    pre_visit(vertex);
    for (auto neighbor : adjacency_[vertex]) {
      if (!visited[neighbor]) {
        DepthFirstVisit(neighbor, visited, pre_visit, post_visit);
      }
    }
    post_visit(vertex);
  }

  /**
   * Обход графа в глубину с возможностью кастомизации обработки вершин.
   * @param pre_visit Действие перед посещением вершины
   * @param post_visit Действие после посещения вершины
   */
  template <typename PreVisit, typename PostVisit>
  void TraverseGraph(PreVisit&& pre_visit, PostVisit&& post_visit) {
    std::vector<bool> visited(adjacency_.size(), false);
    for (size_t v = 0; v < adjacency_.size(); ++v) {
      if (!visited[v]) {
        DepthFirstVisit(v, visited, pre_visit, post_visit);
      }
    }
  }

  /**
   * Обход графа для разметки компонент сильной связности.
   * @param order_stack Стек с порядком обработки вершин
   */
  void MarkComponents(std::stack<size_t>& order_stack) {
    std::vector<bool> visited(adjacency_.size(), false);
    vertex_component_ids_.resize(adjacency_.size());
    current_scc_count_ = 0;

    while (!order_stack.empty()) {
      auto vertex = order_stack.top();
      if (!visited[vertex]) {
        DepthFirstVisit(vertex, visited, [this](auto v) { this->vertex_component_ids_[v] = this->current_scc_count_; }, [](auto) {});
        ++current_scc_count_;
      }
      order_stack.pop();
    }
  }

  /// Парсит целое число из строки
  static int ParseInt(const char** ptr) {
    int sign = 1;
    int value = 0;
    while (**ptr && std::isspace(**ptr)) ++(*ptr);
    if (**ptr == '-') {
      sign = -1;
      ++(*ptr);
    }
    while (**ptr >= '0' && **ptr <= '9') {
      value = value * 10 + (**ptr - '0');
      ++(*ptr);
    }
    return sign * value;
  }

 public:
  Graph() = default;
  explicit Graph(std::vector<std::vector<size_t>>&& adjacency) : adjacency_(std::move(adjacency)) {}

  /// Выводит граф в стандартный поток вывода
  void Print() const {
    for (size_t v = 0; v < adjacency_.size(); ++v) {
      std::cout << v;
      if (!adjacency_[v].empty()) {
        std::cout << "   ";
        for (auto u : adjacency_[v]) {
          std::cout << u << ' ';
        }
      }
      std::cout << kEndOfEdgesMarker << '\n';
    }
  }

  /// Читает граф из стандартного потока ввода
  static Graph ReadFromStdin() noexcept {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::string buffer;
    {
      std::streambuf* sb = std::cin.rdbuf();
      buffer.reserve(1 << 20);  // 1MB
      char c;
      while ((c = sb->sgetc()) != EOF) {
        buffer.push_back(c);
        sb->snextc();
      }
    }

    const char* ptr = buffer.data();
    const char* end = buffer.data() + buffer.size();
    int n = ParseInt(&ptr);
    std::vector<std::vector<std::size_t>> adj(n);
    for (int i = 0; i < n; ++i) {
      int v = ParseInt(&ptr);
      adj[v].reserve(8);
      while (ptr < end) {
        int u = ParseInt(&ptr);
        if (u == -1) break;
        adj[v].push_back(u);
      }
    }
    return Graph(std::move(adj));
  }

  /// Строит обратный граф (с инвертированными рёбрами)
  Graph Reverse() const {
    std::vector<std::vector<size_t>> reversed(adjacency_.size());

    // Предварительный расчёт входящих степеней для оптимизации
    std::vector<size_t> in_degree(adjacency_.size(), 0);
    for (const auto& neighbors : adjacency_) {
      for (auto v : neighbors) ++in_degree[v];
    }
    for (size_t v = 0; v < reversed.size(); ++v) {
      reversed[v].reserve(in_degree[v]);
    }

    // Заполнение обратного графа
    for (size_t u = 0; u < adjacency_.size(); ++u) {
      for (auto v : adjacency_[u]) {
        reversed[v].push_back(u);
      }
    }

    return Graph(std::move(reversed));
  }

  /// Находит все сильно связные компоненты (возвращает массив component_id для вершин)
  std::vector<size_t> FindStronglyConnectedComponents() {
    auto reversed_graph = this->Reverse();
    ClearProcessingStack();

    // Первый обход для определения порядка обработки
    reversed_graph.TraverseGraph([](auto) {}, [this](auto v) { this->processing_order_.push(v); });

    // Второй обход для разметки компонент
    MarkComponents(processing_order_);

    return vertex_component_ids_;
  }

  /// Строит метаграф на основе компонент сильной связности
  Graph BuildMetaGraph() const {
    std::unordered_map<size_t, std::unordered_set<size_t>> meta_edges;

    for (size_t u = 0; u < adjacency_.size(); ++u) {
      auto u_comp = vertex_component_ids_[u];
      for (auto v : adjacency_[u]) {
        auto v_comp = vertex_component_ids_[v];
        if (u_comp != v_comp) {
          meta_edges[u_comp].insert(v_comp);
        }
      }
    }

    // Преобразование в список смежности
    std::vector<std::vector<size_t>> meta_adj(current_scc_count_);
    for (const auto& [comp, neighbors] : meta_edges) {
      meta_adj[comp].assign(neighbors.begin(), neighbors.end());
    }

    return Graph(std::move(meta_adj));
  }
};

int main() {
  auto graph = Graph::ReadFromStdin();
  auto components = graph.FindStronglyConnectedComponents();

  // Группировка вершин по компонентам
  std::unordered_map<size_t, std::vector<size_t>> component_map;
  for (size_t v = 0; v < components.size(); ++v) {
    component_map[components[v]].push_back(v);
  }

  // Вывод компонент
  for (const auto& [comp_id, vertices] : component_map) {
    std::cout << "SCC " << comp_id << ": ";
    for (auto v : vertices) std::cout << v << " ";
    std::cout << "\n";
  }

  // Построение и вывод метаграфа
  auto meta_graph = graph.BuildMetaGraph();
  std::cout << "Meta-graph:\n";
  meta_graph.Print();

  return EXIT_SUCCESS;
}