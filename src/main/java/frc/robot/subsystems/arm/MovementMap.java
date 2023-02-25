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
 * Class that is used to store the map of possible locations for the arm to allow for movement
 * without running into important parts of the robot.
 */
public class MovementMap {

    private final Set<Node<Region>> mainMap = new HashSet<>();

    private static MovementMap instance;

    public static MovementMap getInstance() {
        if (instance == null) {
            instance = new MovementMap();
        }
        return instance;
    }

    private MovementMap() {
        // TODO ADD REGIONS TO MAP
        // Node<Region> node1 = new Node<>(new Region(new Translation3d(0, 0, 0), new
        // Translation3d(0, 0, 0)));
        // Node<Region> node2 = new Node<>(new Region(new Translation3d(0, 0, 0), new
        // Translation3d(0, 0, 0)));
        // node1.addNeighboor(node2);
        // node2.addNeighboor(node1);
        // mainMap.add(node1);
        // mainMap.add(node2);

        Node<Region> region1 = new Node<>(new Region(new Translation3d(0, 0, 0), new Translation3d(1, 2, 0)));
        Node<Region> region2 = new Node<>(new Region(new Translation3d(0, 1, 0), new Translation3d(2, 3, 0)));
        Node<Region> region3 = new Node<>(new Region(new Translation3d(1, 3, 0), new Translation3d(2, 4, 0)));
        Node<Region> region4 = new Node<>(new Region(new Translation3d(0, 4, 0), new Translation3d(3, 5, 0)));
        Node<Region> region5 = new Node<>(new Region(new Translation3d(2, 1, 0), new Translation3d(3, 2, 0)));
        Node<Region> region6 = new Node<>(new Region(new Translation3d(3, 1, 0), new Translation3d(4, 2, 0)));
        Node<Region> region7 = new Node<>(new Region(new Translation3d(3, 4, 0), new Translation3d(4, 5, 0)));
        Node<Region> region8 = new Node<>(new Region(new Translation3d(3, 0, 0), new Translation3d(5, 5, 0)));

        makeNeighbors(region1, region2);
        makeNeighbors(region2, region3);
        makeNeighbors(region2, region5);
        makeNeighbors(region3, region4);
        makeNeighbors(region4, region7);
        makeNeighbors(region7, region8);
        makeNeighbors(region8, region6);
        makeNeighbors(region5, region6);
        makeNeighbors(region4, region2);
        makeNeighbors(region6, region2);

        mainMap.add(region1);
        mainMap.add(region2);
        mainMap.add(region3);
        mainMap.add(region4);
        mainMap.add(region5);
        mainMap.add(region6);
        mainMap.add(region7);
        mainMap.add(region8);
    }

    public Set<Node<Region>> getMainMap() {
        return mainMap;
    }

    private static void makeNeighbors(Node<Region> rg1, Node<Region> rg2) {
        rg1.addNeighboor(rg2);
        rg2.addNeighboor(rg1);
    }

    public static List<Positions.Pose3d> generatePathBetweenTwoPoints(Positions.Pose3d startPose, Positions.Pose3d endPose, Set<Node<Region>> map) {
        Translation3d start = startPose.inRobotSpace().getTranslation();
        Translation3d end = endPose.inRobotSpace().getTranslation();
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

        // Do Dijkstra's algorithm to find the shortest path to a region that contains
        // the end point

        Set<NodeWrapper<Region>> settledNodes = new HashSet<>();
        Set<NodeWrapper<Region>> unsettledNodes = new HashSet<>();

        NodeWrapper<Region> startNodeWrapper = new NodeWrapper<>(startNode, 0);

        unsettledNodes.add(startNodeWrapper);

        while (unsettledNodes.size() != 0) {
            NodeWrapper<Region> currentNode = getLowestDistanceNode(unsettledNodes);
            if (currentNode.node.getData().contains(end)) {
                List<Positions.Pose3d> path = new ArrayList<>();
                for (Node<Region> pathNode : currentNode.getShortestPath()) {
                    path.add(Positions.Pose3d.fromRobotSpace(new Pose3d(pathNode.getData().getCenter(), new Rotation3d())));
                }
                return path;
            }
            unsettledNodes.remove(currentNode);
            for (Node<Region> neighboor : currentNode.node.getNeighbors()) {
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
        for (NodeWrapper<Region> node : unsettledNodes) {
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
        System.out.println(evaluationNode.getDistance());
        if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
            evaluationNode.setDistance(sourceDistance + edgeWeigh);
            LinkedList<Node<Region>> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
            shortestPath.add(sourceNode.getNode());
            System.out.println(shortestPath);
            evaluationNode.setShortestPath(shortestPath);
        }
    }

    private static class NodeWrapper<T> {
        private final Node<T> node;
        private double distance = 10000000;
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
