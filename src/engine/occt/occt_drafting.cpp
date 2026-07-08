// SPDX-License-Identifier: Apache-2.0
//
// occt_drafting.cpp — the OCCT HLR ORACLE (MOAT M-GS GS1, GATE b).
//
// OcctEngine::hlr_project runs OCCT's HLRBRep_Algo + HLRBRep_HLRToShape (the
// orthographic projector) and returns the visible/hidden 2D drawing-plane segment
// sets in the SAME basis the native orthographic_hlr core uses, so the sim parity
// harness (tests/sim/native_hlr_parity.mm) can compare native-vs-OCCT on visible
// count, hidden count, total projected length, and endpoint positions.
//
// Drawing-plane basis (identical to native detail::makeBasis):
//   right  = normalize(viewDir × up)
//   trueUp = right × viewDir
//   a world point P projects to (P·right, P·trueUp).
// The HLRAlgo_Projector is built from a gp_Ax2 whose main (Z) direction is
// −viewDir and whose X direction is `right`; its Y direction is then
// (−viewDir) × right = right × viewDir = trueUp. OCCT flattens the outline into
// this frame, so the output edge coordinates X(), Y() ARE (P·right, P·trueUp),
// and the eye sits on the −viewDir (camera) side so OCCT's visible == native's
// visible. This file is the ONLY HLR-linked piece; the native library never
// includes an OCCT header.

#include "engine/occt/occt_engine.h"

#include <cmath>
#include <vector>

#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace cyber {

namespace {

// Append every straight-sampled sub-segment of the edges in `compound` to `out`,
// reading the flattened 2D outline coordinates (X, Y) OCCT emits in the projector
// frame. A polyhedral outline edge is a single line (2 points ⇒ 1 segment); a
// generic outline curve is discretized like edge_polylines.
void collectSegments(const TopoDS_Shape& compound, std::vector<DrawingSegmentData>& out) {
    if (compound.IsNull()) {
        return;
    }
    for (TopExp_Explorer ex(compound, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (BRep_Tool::Degenerated(edge)) {
            continue;
        }
        BRepAdaptor_Curve curve(edge);
        GCPnts_TangentialDeflection disc(curve, 0.2, 0.2);
        const int n = disc.NbPoints();
        if (n < 2) {
            continue;
        }
        for (int k = 1; k < n; ++k) {
            const gp_Pnt a = disc.Value(k);
            const gp_Pnt b = disc.Value(k + 1);
            out.push_back(DrawingSegmentData{a.X(), a.Y(), b.X(), b.Y()});
        }
    }
}

}  // namespace

Result<DrawingData> OcctEngine::hlr_project(EngineShape body, const double viewDir[3],
                                            const double up[3], HlrOptionsData opts) {
    (void)opts;  // OCCT projects the exact B-rep; no occluder deflection needed.
    const TopoDS_Shape* shape = occt::unwrap(body);
    if (shape == nullptr || shape->IsNull()) {
        return make_error("hlr_project: unknown/empty body");
    }
    if (viewDir == nullptr || up == nullptr) {
        return make_error("hlr_project: null viewDir/up");
    }
    return occt::occtGuard([&]() -> Result<DrawingData> {
        const gp_Vec vd(viewDir[0], viewDir[1], viewDir[2]);
        const gp_Vec uh(up[0], up[1], up[2]);
        if (vd.Magnitude() < 1.0e-12) {
            return make_error("hlr_project: degenerate view direction");
        }
        const gp_Vec rightVec = vd.Crossed(uh);
        if (rightVec.Magnitude() < 1.0e-12) {
            return make_error("hlr_project: up hint parallel to view direction");
        }
        // gp_Ax2(loc, N, Vx): main (Z) = −viewDir (eye on the camera side), X = right;
        // Y = N × X = right × viewDir = trueUp → the native drawing-plane basis.
        const gp_Ax2 ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(vd.Reversed()), gp_Dir(rightVec));
        HLRAlgo_Projector projector(ax2);  // orthographic (parallel) projection

        Handle(HLRBRep_Algo) hlr = new HLRBRep_Algo();
        hlr->Add(*shape);
        hlr->Projector(projector);
        hlr->Update();
        hlr->Hide();

        HLRBRep_HLRToShape toShape(hlr);
        DrawingData out;
        collectSegments(toShape.VCompound(), out.visible);       // visible sharp edges
        collectSegments(toShape.OutLineVCompound(), out.visible);  // + visible smooth outlines
        collectSegments(toShape.HCompound(), out.hidden);        // hidden sharp edges
        collectSegments(toShape.OutLineHCompound(), out.hidden);   // + hidden smooth outlines
        return out;
    });
}

}  // namespace cyber
