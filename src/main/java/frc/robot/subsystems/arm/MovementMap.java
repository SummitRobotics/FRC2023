package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utilities.Node;
import frc.robot.utilities.Positions;
import frc.robot.utilities.Region;

/*
* Class that is used to store the map of possible locations for the arm
* to allow for movement without running into important parts of the robot.
*/
public class MovementMap {

private final Set<Node<Region>> mainMap = new HashSet<>();

public MovementMap() {
    // TODO ADD REGIONS TO MAP
    Node<Region> node1 = new Node<>(new Region(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0)));
    Node<Region> node2 = new Node<>(new Region(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0)));
    node1.addNeighboor(node2);
    node2.addNeighboor(node1);
    mainMap.add(node1);
    mainMap.add(node2);
}

public Set<Node<Region>> getMainMap() {
    return mainMap;
}

public static List<Positions.Pose3d> generatePathBetweenTwoPoints(Positions.Pose3d startPose, Positions.Pose3d endPose, Set<Node<Region>> map) {
    Translation3d start = startPose.inOtherSpace(Arm.ROBOT_TO_TURRET_BASE).getTranslation();
    Translation3d end = endPose.inOtherSpace(Arm.ROBOT_TO_TURRET_BASE).getTranslation();
    Node<Region> startNode = null;
    for (Node<Region> node : map) {
    if (node.getData().contains(start)) {
        startNode = node;
        break;
    }
    }
    if (startNode == null) {
    return null;
    }
    
    // Do Dijkstra's algorithm to find the shortest path to a region that contains the end point

    Set<NodeWrapper<Region>> settledNodes = new HashSet<>();
    Set<NodeWrapper<Region>> unsettledNodes = new HashSet<>();

    NodeWrapper<Region> startNodeWrapper = new NodeWrapper<>(startNode, 0);

    unsettledNodes.add(startNodeWrapper);

    while (unsettledNodes.size() != 0) {
    NodeWrapper<Region> currentNode = getLowestDistanceNode(unsettledNodes);
    if (currentNode.node.getData().contains(end)){
        List<Positions.Pose3d> path = new ArrayList<>();
        for (Node<Region> pathNode: currentNode.getShortestPath()) {
        path.add(Positions.Pose3d.fromOtherSpace(new Pose3d(pathNode.getData().getCenter(), new Rotation3d()), Arm.ROBOT_TO_TURRET_BASE));
        }
        return path;
    }
    unsettledNodes.remove(currentNode);
    for (Node<Region> neighboor: 
        currentNode.node.getNeighbors()) {
        double edgeWeight = currentNode.node.getData().distanceToOther(neighboor.getData());
        NodeWrapper<Region> neighboorNode = new NodeWrapper<>(neighboor, currentNode.distance + edgeWeight);
        if (!settledNodes.contains(neighboorNode)) {
            calculateMinimumDistance(neighboorNode, edgeWeight, currentNode);
            unsettledNodes.add(neighboorNode);
        }
        }
    settledNodes.add(currentNode);
    }
    return null;
}

private static NodeWrapper<Region> getLowestDistanceNode(Set<NodeWrapper<Region>> unsettledNodes) {
    NodeWrapper<Region> lowestDistanceNode = null;
    double lowestDistance = Double.MAX_VALUE;
    for (NodeWrapper<Region> node: unsettledNodes) {
        double nodeDistance = node.getDistance();
        if (nodeDistance < lowestDistance) {
            lowestDistance = nodeDistance;
            lowestDistanceNode = node;
        }
    }
    return lowestDistanceNode;
}

private static void calculateMinimumDistance(NodeWrapper<Region> evaluationNode, double edgeWeigh, NodeWrapper<Region> sourceNode) {
    double sourceDistance = sourceNode.getDistance();
    if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
        evaluationNode.setDistance(sourceDistance + edgeWeigh);
        LinkedList<Node<Region>> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
        shortestPath.add(sourceNode.getNode());
        evaluationNode.setShortestPath(shortestPath);
    }
}

private static class NodeWrapper<T> {
    private final Node<T> node;
    private double distance = Double.MAX_VALUE;
    private LinkedList<Node<T>> shortestPath = new LinkedList<>();

    public NodeWrapper(Node<T> node, double distance) {
    this.node = node;
    this.distance = distance;
    }

    public Node<T> getNode() {
    return node;
    }

    public double getDistance() {
    return distance;
    }

    public void setDistance(double distance) {
    this.distance = distance;
    }

    public LinkedList<Node<T>> getShortestPath() {
    return shortestPath;
    }

    public void setShortestPath(LinkedList<Node<T>> path) {
    this.shortestPath = path;
    }
}
}