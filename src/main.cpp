#include <cstddef>
#include <iostream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Graph {
 private:
  std::size_t curSCC;
  std::vector<std::vector<std::size_t>> adj;
  std::stack<std::size_t> L;
  std::vector<std::size_t> numSCC;

  void clear_stack() { L = std::stack<std::size_t>(); }

  template <typename PreVisit, typename PostVisit>
  void visit(int v, std::vector<bool>& visited, PreVisit&& pre_visit, PostVisit&& post_visit) {
    visited[v] = true;
    pre_visit(v);
    for (const auto& neighbor : adj[v]) {
      if (!visited[neighbor]) {
        visit(neighbor, visited, pre_visit, post_visit);
      }
    }
    post_visit(v);
  }

  template <typename PreVisit, typename PostVisit>
  void DFS(PreVisit&& pre_visit, PostVisit&& post_visit) {
    std::vector<bool> visited(adj.size(), false);
    for (std::size_t v = 0; v < adj.size(); ++v) {
      if (!visited[v]) visit(v, visited, pre_visit, post_visit);
    }
  }

  void DFS(std::stack<std::size_t>& order_stack) {
    std::vector<bool> visited(adj.size(), false);
    numSCC = std::vector<std::size_t>(adj.size());
    curSCC = 0;
    while (!order_stack.empty()) {
      std::size_t v = order_stack.top();
      if (!visited[v]) {
        visit(v, visited, [this](auto v) { this->numSCC[v] = this->curSCC; }, [](auto) {});
        ++curSCC;
      }
      order_stack.pop();
    }
  }

  static int parse_int(const char*& ptr) {
    int sign = 1;
    int value = 0;
    while (*ptr && std::isspace(*ptr)) ++ptr;
    if (*ptr == '-') {
      sign = -1;
      ++ptr;
    }
    while (*ptr >= '0' && *ptr <= '9') {
      value = value * 10 + (*ptr - '0');
      ++ptr;
    }
    return sign * value;
  }

 public:
  Graph() = default;
  Graph(std::vector<std::vector<std::size_t>>&& adjancency_list) : adj(std::move(adjancency_list)) {}

  std::size_t get_size() { return adj.size(); }

  std::stack<std::size_t>& get_L() { return L; }

  void print() const {
    for (std::size_t v = 0; v < adj.size(); ++v) {
      std::cout << v;
      if (!adj[v].empty()) {
        std::cout << "   ";
        for (auto&& u : adj[v]) {
          std::cout << u << ' ';
        }
      }
      std::cout << "   -1" << std::endl;
    }
  }

  static Graph read_from_stdin() noexcept {
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
    int n = parse_int(ptr);
    std::vector<std::vector<std::size_t>> adj(n);
    for (int i = 0; i < n; ++i) {
      int v = parse_int(ptr);
      adj[v].reserve(8);
      while (ptr < end) {
        int u = parse_int(ptr);
        if (u == -1) break;
        adj[v].push_back(u);
      }
    }
    return Graph(std::move(adj));
  }

  Graph reverse() const noexcept {
    std::vector<std::vector<std::size_t>> reverse_adj(adj.size());
    std::vector<std::size_t> in_degree(adj.size(), 0);
    for (const auto& neighbors : adj) {
      for (int v : neighbors) {
        in_degree[v]++;
      }
    }
    for (std::size_t v = 0; v < adj.size(); ++v) {
      reverse_adj[v].reserve(in_degree[v]);
    }
    for (std::size_t u = 0; u < adj.size(); ++u) {
      for (const auto& v : adj[u]) {
        reverse_adj[v].push_back(u);
      }
    }
    return Graph(std::move(reverse_adj));
  }

  std::vector<std::size_t>& find_scc() {
    Graph transported_graph = this->reverse();
    this->clear_stack();
    transported_graph.DFS([](auto) {}, [this](auto v) { this->L.push(v); });
    DFS(L);
    return numSCC;
  }

  Graph build_meta_graph() {
    std::unordered_map<int, std::unordered_set<int>> meta_adj;
    for (std::size_t u = 0; u < adj.size(); ++u) {
      int u_comp = numSCC[u];
      for (auto&& v : adj[u]) {
        int v_comp = numSCC[v];
        if (u_comp != v_comp) {
          meta_adj[u_comp].insert(v_comp);
        }
      }
    }
    std::vector<std::vector<std::size_t>> meta_adj_list(curSCC);
    for (auto&& [comp, neighbors] : meta_adj) {
      meta_adj_list[comp].assign(neighbors.begin(), neighbors.end());
    }
    return Graph(std::move(meta_adj_list));
  }
};

int main() {
  Graph g = Graph::read_from_stdin();
  auto& scc = g.find_scc();
  std::unordered_map<int, std::vector<std::size_t>> components;
  for (std::size_t v = 0; v < scc.size(); ++v) {
    components[scc[v]].push_back(v);
  }
  for (auto&& [comp, nodes] : components) {
    std::cout << "SCC " << comp << ":   ";
    for (auto&& node : nodes) {
      std::cout << node << " ";
    }
    std::cout << std::endl;
  }
  Graph meta = g.build_meta_graph();
  std::cout << "Meta-graph:\n";
  meta.print();
  return EXIT_SUCCESS;
}