package frc.robot.utilities;

import java.util.HashSet;
import java.util.Set;

public class Node<T> {
    private final T data;
    private Set<Node<T>> neighbors = new HashSet<>();

    public Node(T data) {
        this.data = data;
    }

    public T getData() {
        return data;
    }

    public Set<Node<T>> getNeighbors() {
        return neighbors;
    }

    public void addNeighboor(Node<T> node) {
        neighbors.add(node);
    }

    @Override
    public String toString() {
        return "Node: (" + data.toString() + ")";
    }
}
