var Heap       = require('heap');
var Util       = require('../core/Util');
var Heuristic  = require('../core/Heuristic');
var DiagonalMovement = require('../core/DiagonalMovement');

/**
 * Potential fields-based path-finder. Based on AStarFinder.js.
 * @constructor
 * @param {Object} opt
 * @param {boolean} opt.allowDiagonal Whether diagonal movement is allowed.
 *     Deprecated, use diagonalMovement instead.
 * @param {boolean} opt.dontCrossCorners Disallow diagonal movement touching
 *     block corners. Deprecated, use diagonalMovement instead.
 * @param {DiagonalMovement} opt.diagonalMovement Allowed diagonal movement.
 * @param {function} opt.heuristic Heuristic function to estimate the distance
 *     (defaults to manhattan).
 * @param {number} opt.weight Weight to apply to the heuristic to allow for
 *     suboptimal paths, in order to speed up the search.
 */
function PotentialFieldFinder(opt) {
    opt = opt || {};
    this.allowDiagonal = opt.allowDiagonal;
    this.dontCrossCorners = opt.dontCrossCorners;
    this.heuristic = opt.heuristic || Heuristic.manhattan;
    this.weight = opt.weight || 1;
    this.diagonalMovement = opt.diagonalMovement;

    if (!this.diagonalMovement) {
        if (!this.allowDiagonal) {
            this.diagonalMovement = DiagonalMovement.Never;
        } else {
            if (this.dontCrossCorners) {
                this.diagonalMovement = DiagonalMovement.OnlyWhenNoObstacles;
            } else {
                this.diagonalMovement = DiagonalMovement.IfAtMostOneObstacle;
            }
        }
    }

    // When diagonal movement is allowed the manhattan heuristic is not
    //admissible. It should be octile instead
    if (this.diagonalMovement === DiagonalMovement.Never) {
        this.heuristic = opt.heuristic || Heuristic.manhattan;
    } else {
        this.heuristic = opt.heuristic || Heuristic.octile;
    }
}

/**
 * Find and return the the path.
 * @return {Array<Array<number>>} The path, including both start and
 *     end positions.
 */
PotentialFieldFinder.prototype.findPath = function(startX, startY, endX, endY, grid) {
    var startNode = grid.getNodeAt(startX, startY), endNode = grid.getNodeAt(endX, endY);
    var current = startNode, neighbors;
    var path = [[startX, startY]];
    this.addFieldtoGrid(endNode, grid);

    startNode.opened = true;

    while (current !== endNode) {
        neighbors = grid.getNeighbors(current, this.diagonalMovement);
        neighbors = neighbors.sort(function (a, b) {
            return b.field - a.field;
        }).filter(function (node) {
            return !node.opened;
        });

        if (neighbors.length === 0) {
            // Call it an infinite loop and bail
            // fail to find the path
            return [];
        }

        path.push([neighbors[0].x, neighbors[0].y]);
        neighbors[0].opened = true;
        current = neighbors[0];
    }

    return path;
};

PotentialFieldFinder.prototype.addFieldtoGrid = function (endNode, grid) {
    var node, dx, dy, neighbors;
    for (var y = 0; y < grid.height; y++) {
        for (var x = 0; x < grid.width; x++) {
            node = grid.getNodeAt(x, y);
            dx = Math.abs(endNode.x - node.x);
            dy = Math.abs(endNode.y - node.y);
            node.field = -this.heuristic(dx, dy);

            // Add number of walkable neighbors
            neighbors = grid.getNeighbors(node, this.diagonalMovement);
            node.field += neighbors.length;
        }
    }
}

module.exports = PotentialFieldFinder;
