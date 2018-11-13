import heapq


class PriorityQueue:
    def __init__(self):
        # type: () -> object
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

    def get_items(self):
        l = []
        copy = self.elements
        for ele in len(copy):
            l.append(heapq.heappop(copy)[1])

        return l