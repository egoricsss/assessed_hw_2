#include <algorithm>
#include <cstddef>
#include <iostream>
#include <stack>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <map>
#include <set>
#include <vector>

using VertexId = size_t;

// Константа для обозначения конца списка рёбер при выводе
constexpr std::string kEndOfEdgesMarker = "   -1";

/**
 * Класс для представления ориентированного графа и работы с ним.
 * Поддерживает операции поиска сильно связных компонент (SCC),
 * построения обратного графа и метаграфа.
 */
class Graph {
    private:
	using AdjacencyList = std::vector<std::vector<VertexId> >;

	VertexId current_scc_count_; // Текущее количество SCC
	AdjacencyList adjacency_; // Список смежности
	std::stack<VertexId> processing_order_; // Порядок обработки вершин
	std::vector<VertexId> vertex_component_ids_; // ID компонент для вершин

	/// Очищает стек порядка обработки
	void ClearProcessingStack()
	{
		std::stack<VertexId>().swap(processing_order_);
	}

	/**
   * Рекурсивный обход в глубину с обработкой перед и после посещения вершины.
   * @param vertex Текущая вершина
   * @param visited Массив посещённых вершин
   * @param pre_visit Действие перед посещением вершины
   * @param post_visit Действие после посещения вершины
   */
	template <typename PreVisit, typename PostVisit>
	void DepthFirstVisit(VertexId vertex, std::vector<char> &visited,
			     PreVisit &&pre_visit, PostVisit &&post_visit)
	{
		visited[vertex] = true;
		pre_visit(vertex);
		for (auto neighbor : adjacency_[vertex]) {
			if (!visited[neighbor]) {
				DepthFirstVisit(neighbor, visited, pre_visit,
						post_visit);
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
	void TraverseGraph(PreVisit &&pre_visit, PostVisit &&post_visit)
	{
		std::vector<char> visited(adjacency_.size(), false);
		for (VertexId v = 0; v < adjacency_.size(); ++v) {
			if (!visited[v]) {
				DepthFirstVisit(v, visited, pre_visit,
						post_visit);
			}
		}
	}

	/**
   * Обход графа для разметки компонент сильной связности.
   * @param order_stack Стек с порядком обработки вершин
   */
	void MarkComponents(std::stack<VertexId> &order_stack)
	{
		std::vector<char> visited(adjacency_.size(), false);
		vertex_component_ids_.resize(adjacency_.size());
		current_scc_count_ = 0;

		while (!order_stack.empty()) {
			auto vertex = order_stack.top();
			if (!visited[vertex]) {
				DepthFirstVisit(
					vertex, visited,
					[this](auto v) {
						this->vertex_component_ids_[v] =
							this->current_scc_count_;
					},
					[](auto) {});
				++current_scc_count_;
			}
			order_stack.pop();
		}
	}

	/// Парсит целое число из строки
	static int ParseInt(const char **ptr)
	{
		if (!ptr || !*ptr)
			return 0;

		int sign = 1;
		int value = 0;

		// Пропуск пробельных символов
		while (**ptr && std::isspace(**ptr))
			++(*ptr);

		// Обработка знака
		if (**ptr == '-') {
			sign = -1;
			++(*ptr);
		}

		// Проверка, что следующий символ - цифра
		if (**ptr < '0' || **ptr > '9') {
			throw std::runtime_error("Invalid number format");
		}

		// Парсинг числа
		while (**ptr >= '0' && **ptr <= '9') {
			value = value * 10 + (**ptr - '0');
			++(*ptr);
		}

		return sign * value;
	}

	/// Читает граф из буфера
	static AdjacencyList ParseGraphBuffer(const std::string &buffer)
	{
		const char *ptr = buffer.data();
		const char *end = buffer.data() + buffer.size();

		// Парсинг количества вершин
		int n = ParseInt(&ptr);

		// Проверка на корректность
		if (n <= 0 || ptr >= end) {
			throw std::runtime_error("Invalid input format");
		}

		AdjacencyList adj(n);

		for (int i = 0; i < n; ++i) {
			int v = ParseInt(&ptr);

			// Проверка на корректность номера вершины
			if (v < 0 || v >= n) {
				throw std::runtime_error(
					"Invalid vertex index");
			}

			adj[v].reserve(8);
			int u;

			while (ptr < end && (u = ParseInt(&ptr)) != -1) {
				// Проверка на корректность номера смежной вершины
				if (u < 0 || u >= n) {
					throw std::runtime_error(
						"Invalid adjacent vertex index");
				}

				adj[v].push_back(u);
			}
		}

		return adj;
	}

    public:
	Graph() = default;
	explicit Graph(AdjacencyList &&adjacency)
		: adjacency_(std::move(adjacency))
	{
	}

	/// Выводит граф в стандартный поток вывода
	void Print() const
	{
		for (VertexId v = 0; v < adjacency_.size(); ++v) {
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
	static Graph ReadFromStdin()
	{
		std::ios::sync_with_stdio(false);
		std::cin.tie(nullptr);

		std::string buffer;
		buffer.reserve(1 << 20); // 1MB

		// Чтение всего входа в буфер
		char c;
		while (std::cin.get(c)) {
			buffer.push_back(c);
		}

		return Graph(ParseGraphBuffer(buffer));
	}

	/// Строит обратный граф (с инвертированными рёбрами)
	Graph Reverse() const
	{
		AdjacencyList reversed(adjacency_.size());

		// Предварительный расчёт входящих степеней для оптимизации
		std::vector<size_t> in_degree(adjacency_.size(), 0);
		for (const auto &neighbors : adjacency_) {
			for (auto v : neighbors)
				++in_degree[v];
		}
		for (VertexId v = 0; v < reversed.size(); ++v) {
			reversed[v].reserve(in_degree[v]);
		}

		// Заполнение обратного графа
		for (VertexId u = 0; u < adjacency_.size(); ++u) {
			for (auto v : adjacency_[u]) {
				reversed[v].push_back(u);
			}
		}

		return Graph(std::move(reversed));
	}

	/// Находит все сильно связные компоненты (возвращает массив component_id для вершин)
	std::vector<VertexId> FindStronglyConnectedComponents()
	{
		auto reversed_graph = this->Reverse();
		ClearProcessingStack();

		// Первый обход для определения порядка обработки
		reversed_graph.TraverseGraph(
			[](auto) {},
			[this](auto v) { this->processing_order_.push(v); });

		// Второй обход для разметки компонент
		MarkComponents(processing_order_);

		return vertex_component_ids_;
	}

	/// Строит метаграф на основе компонент сильной связности
	Graph BuildMetaGraph() const
	{
		// Используем map для упорядоченного вывода
		std::map<VertexId, std::set<VertexId> > meta_edges;

		for (VertexId u = 0; u < adjacency_.size(); ++u) {
			auto u_comp = vertex_component_ids_[u];
			for (auto v : adjacency_[u]) {
				auto v_comp = vertex_component_ids_[v];
				if (u_comp != v_comp) {
					meta_edges[u_comp].insert(v_comp);
				}
			}
		}

		// Преобразование в список смежности
		AdjacencyList meta_adj(current_scc_count_);
		for (const auto &[comp, neighbors] : meta_edges) {
			meta_adj[comp].assign(neighbors.begin(),
					      neighbors.end());
		}

		return Graph(std::move(meta_adj));
	}
};

int main()
{
	try {
		auto graph = Graph::ReadFromStdin();
		auto components = graph.FindStronglyConnectedComponents();

		// Группировка вершин по компонентам
		std::unordered_map<VertexId, std::vector<VertexId> >
			component_map;
		for (VertexId v = 0; v < components.size(); ++v) {
			component_map[components[v]].push_back(v);
		}

		// Вывод компонент
		for (const auto &[comp_id, vertices] : component_map) {
			std::cout << "SCC " << comp_id << ": ";
			for (auto v : vertices)
				std::cout << v << " ";
			std::cout << "\n";
		}

		// Построение и вывод метаграфа
		auto meta_graph = graph.BuildMetaGraph();
		std::cout << "Meta-graph:\n";
		meta_graph.Print();
	} catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}