# SPDX-License-Identifier: LGPL-2.1-or-later

import json
import os
import sys
import tempfile
import time
import unittest
from contextlib import contextmanager

import FreeCAD
import Part
import TestSketcherApp
from PySide import QtCore

from .TechDrawTestUtilities import createPageWithSVGTemplate

try:
    import resource
except ImportError:
    resource = None


def peak_rss_kib():
    if resource is None:
        return None
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return peak // 1024 if sys.platform == "darwin" else peak


class WorkflowBenchmarkTest(unittest.TestCase):
    """Conformance and opt-in metrics for one representative CAD workflow."""

    def setUp(self):
        self.document = FreeCAD.newDocument("WorkflowBenchmark")
        self.metrics = {}
        self.temp_dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        if os.environ.get("FREECAD_WORKFLOW_BENCHMARK"):
            print("WORKFLOW_BENCHMARK " + json.dumps(self.metrics, sort_keys=True))
        self.temp_dir.cleanup()
        FreeCAD.closeDocument(self.document.Name)

    @contextmanager
    def stage(self, name):
        started = time.perf_counter()
        rss_before = peak_rss_kib()
        try:
            yield
        finally:
            rss_after = peak_rss_kib()
            result = {"seconds": round(time.perf_counter() - started, 6)}
            if rss_after is not None:
                result["peak_rss_kib"] = rss_after
                result["peak_rss_delta_kib"] = max(0, rss_after - rss_before)
            self.metrics[name] = result

    def assertKernelEquivalent(self, reference, candidate, tolerance=1e-6):
        """The sole kernel experiment seam: validity, topology, and material."""
        for shape in (reference, candidate):
            self.assertFalse(shape.isNull())
            self.assertTrue(shape.isValid())
            self.assertEqual(len(shape.Solids), 1)
        self.assertEqual(
            tuple(len(getattr(reference, kind)) for kind in ("Faces", "Edges", "Vertexes")),
            tuple(len(getattr(candidate, kind)) for kind in ("Faces", "Edges", "Vertexes")),
        )
        self.assertAlmostEqual(reference.Volume, candidate.Volume, delta=tolerance)
        self.assertLess(reference.cut(candidate).Volume, tolerance)
        self.assertLess(candidate.cut(reference).Volume, tolerance)

    @staticmethod
    def waitForView():
        loop = QtCore.QEventLoop()
        timer = QtCore.QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start(2000)
        loop.exec_()

    def testSketchEditStepTechDraw(self):
        with self.stage("sketch_features"):
            body = self.document.addObject("PartDesign::Body", "Body")
            pad_sketch = body.newObject("Sketcher::SketchObject", "PadSketch")
            TestSketcherApp.CreateRectangleSketch(pad_sketch, (0, 0), (20, 10))
            pad = body.newObject("PartDesign::Pad", "Pad")
            pad.Profile = pad_sketch
            pad.Length = 8
            pad.Reversed = True

            pocket_sketch = body.newObject("Sketcher::SketchObject", "PocketSketch")
            TestSketcherApp.CreateRectangleSketch(pocket_sketch, (7, 3), (6, 4))
            pocket = body.newObject("PartDesign::Pocket", "Pocket")
            pocket.Profile = pocket_sketch
            pocket.Length = 3
            self.document.recompute()

        self.assertTrue(pocket.Shape.isValid())
        self.assertAlmostEqual(pocket.Shape.Volume, 1528.0, delta=1e-7)

        with self.stage("parametric_edit"):
            pad.Length = 10
            pocket.Length = 4
            self.document.recompute()

        self.assertTrue(pocket.Shape.isValid())
        self.assertAlmostEqual(pocket.Shape.Volume, 1904.0, delta=1e-7)
        edited_shape = pocket.Shape.copy()

        with self.stage("step_round_trip"):
            step_path = os.path.join(self.temp_dir.name, "workflow.step")
            edited_shape.exportStep(step_path)
            step_shape = Part.read(step_path)

        self.assertGreater(os.path.getsize(step_path), 0)
        self.assertKernelEquivalent(edited_shape, step_shape)

        with self.stage("techdraw_projection"):
            imported = self.document.addObject("Part::Feature", "StepResult")
            imported.Shape = step_shape
            page = createPageWithSVGTemplate(self.document)
            view = self.document.addObject("TechDraw::DrawViewPart", "WorkflowView")
            page.addView(view)
            view.Source = [imported]
            view.Direction = (0.0, 0.0, 1.0)
            self.document.recompute()
            self.waitForView()

        self.assertIn("Up-to-date", view.State)
        self.assertEqual(len(view.getVisibleEdges()), 8)


if __name__ == "__main__":
    unittest.main()
