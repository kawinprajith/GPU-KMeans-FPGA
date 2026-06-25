// =============================================================
//  distance_pe.v  –  Processing Element (Combinational)
//
//  Computes squared Euclidean distance from point (xi,yi) to
//  both centroids (cx0,cy0) and (cx1,cy1), then assigns the
//  point to the nearer cluster.
//
//  dist = (xi-cxN)^2 + (yi-cyN)^2   (integer, no sqrt needed)
//  cluster_id = 0 if dist0 <= dist1, else 1
//
//  Target : xc7z020clg400-1 (Zynq-7000)
// =============================================================
`timescale 1ns / 1ps

module distance_pe #(
    parameter DATA_W = 8,          // coordinate bit-width
    parameter DIST_W = 18          // squared-distance bit-width
)(
    // Point coordinates
    input  [DATA_W-1:0] xi,
    input  [DATA_W-1:0] yi,
    // Centroid 0
    input  [DATA_W-1:0] cx0,
    input  [DATA_W-1:0] cy0,
    // Centroid 1
    input  [DATA_W-1:0] cx1,
    input  [DATA_W-1:0] cy1,
    // Outputs
    output              cluster_id, // 0 = closer to centroid 0, 1 = closer to centroid 1
    output [DIST_W-1:0] dist0,      // squared distance to centroid 0 (debug)
    output [DIST_W-1:0] dist1       // squared distance to centroid 1 (debug)
);

    // --------------------------------------------------------
    // Absolute differences  (DATA_W+1 bits, unsigned)
    // Using abs avoids signed-arithmetic issues in plain Verilog
    // --------------------------------------------------------
    wire [DATA_W:0] absdx0, absdy0;
    wire [DATA_W:0] absdx1, absdy1;

    assign absdx0 = (xi >= cx0) ? ({1'b0,xi} - {1'b0,cx0}) : ({1'b0,cx0} - {1'b0,xi});
    assign absdy0 = (yi >= cy0) ? ({1'b0,yi} - {1'b0,cy0}) : ({1'b0,cy0} - {1'b0,yi});
    assign absdx1 = (xi >= cx1) ? ({1'b0,xi} - {1'b0,cx1}) : ({1'b0,cx1} - {1'b0,xi});
    assign absdy1 = (yi >= cy1) ? ({1'b0,yi} - {1'b0,cy1}) : ({1'b0,cy1} - {1'b0,yi});

    // --------------------------------------------------------
    // Squared distances  (DIST_W bits)
    // max = 255^2 + 255^2 = 130050 < 2^17  → fits in 18 bits
    // --------------------------------------------------------
    assign dist0 = (absdx0 * absdx0) + (absdy0 * absdy0);
    assign dist1 = (absdx1 * absdx1) + (absdy1 * absdy1);

    // --------------------------------------------------------
    // Cluster assignment
    // Tie-break: equal distance → cluster 0
    // --------------------------------------------------------
    assign cluster_id = (dist1 < dist0) ? 1'b1 : 1'b0;

endmodule
