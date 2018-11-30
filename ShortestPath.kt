import java.util.*

fun run(mapString: String): String {
    val graph = Graph(mapString)
    val engine = Dijkstra()
    engine.readGraph(graph.numberOfNodes, graph.edges)
    val path = engine.shortestPath(graph.startIndex, graph.targetIndex)
    //println("\n\n")
    //println(mapString)
    //println("\n\n")
    val result = graph.getResultString(path)
    //println(result)
    return result
}

class Dijkstra {

    private lateinit var adjacents: MutableList<MutableList<Int>>
    private lateinit var costs: MutableList<MutableList<Double>>
    private lateinit var parents: IntArray
    private lateinit var visited: BooleanArray
    private lateinit var distances: DoubleArray

    private lateinit var queue: PriorityQueue<Entry>
    private fun getHeadPQueue(): Int {
        val head = queue.poll()
        return head?.index ?: -1
    }

    private data class Entry(
        var index: Int,
        var distance: Double
    ) : Comparable<Entry> {
        override fun compareTo(other: Entry): Int {
            return when {
                (other.distance > this.distance) -> -1
                (other.distance < this.distance) -> 1
                else -> 0
            }
        }
    }

    fun readGraph(numberOfNodes: Int, edges: List<Graph.Edge>) {
        adjacents = mutableListOf()
        costs = mutableListOf()

        repeat(numberOfNodes) {
            adjacents.add(mutableListOf())
            costs.add(mutableListOf())
        }

        parents = IntArray(numberOfNodes)
        visited = BooleanArray(numberOfNodes)
        distances = DoubleArray(numberOfNodes)
        queue = PriorityQueue()

        edges.forEach {
            adjacents[it.from].add(it.to)
            costs[it.from].add(it.weight)
        }

        repeat(distances.size) {
            distances[it] = Double.MAX_VALUE
            parents[it] = -1
            val entry = Entry(it, distances[it])
            queue.add(entry)
        }
    }

    fun shortestPath(s: Int, t: Int): List<Int> {
        distances[s] = 0.0
        val entry = Entry(s, 0.0)
        queue.add(entry)

        while (true) {
            var minimus: Int
            while (true) {
                minimus = getHeadPQueue()
                if (minimus != -1 && distances[minimus] == Double.MAX_VALUE) minimus = -1
                if (minimus == -1 || !visited[minimus]) break
            }

            if (minimus == -1) break
            visited[minimus] = true

            for (it in 0 until adjacents[minimus].size) {
                val examined = adjacents[minimus][it]
                val distToExamined = costs[minimus][it]
                if (distances[examined] > distances[minimus] + distToExamined) {
                    //update
                    val update = distances[minimus] + distToExamined
                    distances[examined] = update
                    parents[examined] = minimus
                    queue.add(Entry(examined, update))
                }
            }
        }
        return if (distances[t] == Double.MAX_VALUE)
            listOf()
        else
            traverseShortestPath(t, s)
    }

    private fun traverseShortestPath(target: Int, start: Int): List<Int> {
        val path = mutableListOf<Int>()
        var node = target
        while (true) {
            path.add(node)
            if (node == start) break
            else node = parents[node]
        }
        return path
    }
}

class Graph(mapString: String) {

    private var xSize: Int = 0
    private var ySize: Int = 0
    var startIndex = 0 // in 'continuous' universe
    var targetIndex = 0 // in 'continuous' universe
    var numberOfNodes = 0
    private val nonRoutable = setOf('\n', 'B')
    private fun Char.isRoutable() = !nonRoutable.contains(this)
                                                //1.
    private val diagX = listOf(-1, 1, 1, -1)    // \/
    private val diagY = listOf(-1, -1, 1, 1)    // /\
    
                                                // 1.
    private val horVerX = listOf(0, 1, 0, -1)   // |
                                                //- -
    private val horVerY = listOf(-1, 0, 1, 0)   // |
    
    private val routableIndices = mutableListOf<Int>()
    // The purpose of these maps is to save some memory as we do not need to store 'B' in graph.
    // I call it 'continuous' universe
    private val rawIndicesToContinuos = mutableMapOf<Int, Int>()
    private val continuosIndicesToRaw = mutableMapOf<Int, Int>()
    val edges = mutableListOf<Edge>()
    private var mapString: String

    init {
        this.mapString = mapString
        var firstNewlineSeen = false
        var rawIndex = 0
        
        mapString.forEach {
            if (!firstNewlineSeen && it != '\n') xSize++
            else if (it == '\n') {
                firstNewlineSeen = true
                ySize++
            }
            if (it == 'S') startIndex = rawIndex
            if (it == 'X') targetIndex = rawIndex
            if (it.isRoutable()) routableIndices.add(rawIndex)
            rawIndex++
        }

        if (xSize > 0) ySize++ // last line does not have the newline char

        // build 'continuous' universe
        for (i in routableIndices.indices) {
            rawIndicesToContinuos.put(routableIndices[i], i)
            continuosIndicesToRaw.put(i, routableIndices[i])
        }
        // convert to 'continuous' universe
        startIndex = rawIndicesToContinuos[startIndex] ?: -1
        targetIndex = rawIndicesToContinuos[targetIndex] ?: -1
        // not very idiomatic...
        if (startIndex == -1 || targetIndex == -1) throw RuntimeException("continuous not working 1")

        this.numberOfNodes = rawIndicesToContinuos.size

        routableIndices.forEach {
            edges.addAll(getEdges(it, diagX, diagY, 1.5))
            edges.addAll(getEdges(it, horVerX, horVerY, 1.0))
        }
    }

    data class Edge(val from: Int, val to: Int, val weight: Double)

    private fun getEdges(
        routableIndex: Int, xDelta: List<Int>,
        yDelta: List<Int>, weight: Double
    ): List<Edge> {
        val edges = mutableListOf<Edge>()
        val coordsFrom = convertToCoords(routableIndex)
        for (i in xDelta.indices) {
            val coordsTo = Pair(coordsFrom.first + xDelta[i], coordsFrom.second + yDelta[i])
            if (isWithinEnvelope(coordsTo) && mapString[convertToIndex(coordsTo)].isRoutable()) {
                // now it is sure that from and to belong to 'continuous' universe
                val from = routableIndex
                val to = convertToIndex(coordsTo)
                // convert to 'continuous'
                val fromContinuous: Int = rawIndicesToContinuos[from] ?: -1
                val toContinuous: Int = rawIndicesToContinuos[to] ?: -1
                // not very idiomatic...
                if (fromContinuous == -1 || toContinuous == -1) throw RuntimeException("continuous not working 2")
                edges.add(Edge(fromContinuous, toContinuous, weight))
            }
        }
        return edges
    }

    // coords(x,y), (xSize + 1) + 1 is hidden newline
    private fun convertToCoords(index: Int): Pair<Int, Int> {
        return Pair(index % (xSize + 1), index / (xSize + 1))
    }

    // coords(x,y), (xSize + 1) + 1 is hidden newline
    private fun convertToIndex(coords: Pair<Int, Int>): Int {
        return (coords.second * (xSize + 1)) + coords.first
    }

    // coords(x,y)
    private fun isWithinEnvelope(coords: Pair<Int, Int>): Boolean {
        return (coords.first in 0 until xSize) && (coords.second in 0 until ySize)
    }

    fun getResultString(nodesContinuousUniverse: List<Int>): String {
        val resultStringArray = mapString.toCharArray()
        nodesContinuousUniverse.forEach {
            val rawIndex = continuosIndicesToRaw[it] ?: -1
            if (rawIndex == -1) throw RuntimeException("continuous not working 3")
            resultStringArray[rawIndex] = '*'
        }
        return String(resultStringArray)
    }
}

