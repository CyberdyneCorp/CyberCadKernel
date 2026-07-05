// SPDX-License-Identifier: Apache-2.0
//
// occt_shapefix.cpp — the OCCT shape-healing oracle / fallback (Phase 4 #4
// `native-healing`).
//
// Implements cyber::occt::sewAndFix: sew a TopoDS_Compound face soup with
// BRepBuilderAPI_Sewing at the caller's tolerance, then heal the resulting shell
// with ShapeFix_Shell (orientation) → ShapeFix_Solid (make a valid solid), and
// report watertight / validSolid / enclosed-volume so the sim parity harness can
// compare it against the native HealResult. This is the DEFERRAL TARGET of the
// engine-internal native-heal hook (native → self-verify → here) AND the ORACLE the
// native healer is measured against.
//
// OCCT is confined to this adapter TU; src/native/** never includes an OCCT header.
// Compiles only under CYBERCAD_HAS_OCCT (iOS / macOS OCCT builds); the host build
// never sees this file (the CMake glob adds src/engine/occt/* only when OCCT is on).
//
#include "engine/occt/occt_engine.h"

#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <ShapeFix_Shell.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeAnalysis_Shell.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

namespace cyber::occt {

SewFixResult sewAndFix(const TopoDS_Shape& faceSoup, double tolerance) {
    SewFixResult out;
    return occtGuard([&]() -> SewFixResult {
        // 1. SEW the face soup: coincident-within-tolerance boundaries become shared
        //    edges. Non-manifold off; the tolerance is the caller's (never widened).
        BRepBuilderAPI_Sewing sewer(tolerance);
        sewer.SetNonManifoldMode(Standard_False);
        for (TopExp_Explorer ex(faceSoup, TopAbs_FACE); ex.More(); ex.Next())
            sewer.Add(ex.Current());
        sewer.Perform();
        const TopoDS_Shape sewn = sewer.SewedShape();
        if (sewn.IsNull()) return out;

        // 2. Take the first shell of the sewn result and check for free (boundary)
        //    edges — none ⇒ closed/watertight (the same criterion the native healer's
        //    isWatertight asserts on the tessellated shell).
        TopoDS_Shell shell;
        for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) {
            shell = TopoDS::Shell(ex.Current());
            break;
        }
        if (!shell.IsNull()) {
            ShapeAnalysis_Shell ana;
            ana.CheckOrientedShells(shell, Standard_True /*alsoInfo*/);
            out.watertight = !ana.HasFreeEdges();

            // 3. ShapeFix the shell (orientation) then make + fix a valid solid.
            ShapeFix_Shell fixShell(shell);
            fixShell.Perform();
            const TopoDS_Shell fixed = fixShell.Shell();

            BRepBuilderAPI_MakeSolid mk(fixed.IsNull() ? shell : fixed);
            if (mk.IsDone()) {
                ShapeFix_Solid fixSolid;
                const TopoDS_Shape solid = fixSolid.SolidFromShell(
                    fixed.IsNull() ? shell : fixed);
                if (!solid.IsNull()) {
                    out.shape = solid;
                    // BRepCheck_Analyzer::IsValid — the SAME gate cyber::occt::isValid
                    // uses (occt_engine.cpp), inlined here so this oracle TU depends on
                    // no other OCCT adapter TU (keeps the sim harness link set minimal).
                    out.validSolid = BRepCheck_Analyzer(solid).IsValid() == Standard_True;
                    GProp_GProps props;
                    BRepGProp::VolumeProperties(solid, props);
                    out.volume = props.Mass();
                    return out;
                }
            }
        }
        out.shape = sewn;  // sewing produced something, but no valid closed solid
        return out;
    });
}

}  // namespace cyber::occt
