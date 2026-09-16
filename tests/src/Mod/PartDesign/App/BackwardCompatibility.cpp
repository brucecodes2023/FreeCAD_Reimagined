// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include "src/App/InitApplication.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <App/Application.h>
#include <App/Document.h>
#include <App/Expression.h>
#include <App/ObjectIdentifier.h>
#include <Base/Console.h>
#include <Base/Interpreter.h>
#include <Mod/PartDesign/App/Body.h>
#include <Mod/PartDesign/App/FeatureChamfer.h>
#include <Mod/PartDesign/App/FeaturePad.h>
#include <Mod/PartDesign/App/FeatureRevolution.h>

// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)

namespace
{

class MigrationLogger final: public Base::ILogger
{
public:
    void sendLog(
        const std::string&,
        const std::string& message,
        Base::LogStyle level,
        Base::IntendedRecipient,
        Base::ContentType
    ) override
    {
        if (level == Base::LogStyle::Warning) {
            warnings.push_back(message);
        }
    }

    const char* name() override
    {
        return "MigrationLogger";
    }

    std::vector<std::string> warnings;
};

class ScopedConsoleObserver
{
public:
    explicit ScopedConsoleObserver(Base::ILogger& logger)
        : logger(logger)
    {
        Base::Console().attachObserver(&logger);
    }

    ~ScopedConsoleObserver()
    {
        Base::Console().detachObserver(&logger);
    }

private:
    Base::ILogger& logger;
};

struct GeometryFingerprint
{
    double volume;
    Base::BoundBox3d bounds;
    std::array<int, 4> topology;
};

GeometryFingerprint geometryFingerprint(const PartDesign::Chamfer& chamfer)
{
    const auto& shape = chamfer.Shape.getValue();
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);

    std::array<int, 4> counts {};
    const std::array<TopAbs_ShapeEnum, 4> types {
        TopAbs_SOLID,
        TopAbs_FACE,
        TopAbs_EDGE,
        TopAbs_VERTEX,
    };
    for (std::size_t i = 0; i < types.size(); ++i) {
        for (TopExp_Explorer explorer(shape, types[i]); explorer.More(); explorer.Next()) {
            ++counts[i];
        }
    }

    return {properties.Mass(), chamfer.Shape.getBoundingBox(), counts};
}

void expectSameGeometry(const GeometryFingerprint& expected, const PartDesign::Chamfer& chamfer)
{
    const auto actual = geometryFingerprint(chamfer);
    EXPECT_NEAR(actual.volume, expected.volume, 1e-7);
    EXPECT_EQ(actual.topology, expected.topology);
    EXPECT_NEAR(actual.bounds.MinX, expected.bounds.MinX, 1e-7);
    EXPECT_NEAR(actual.bounds.MinY, expected.bounds.MinY, 1e-7);
    EXPECT_NEAR(actual.bounds.MinZ, expected.bounds.MinZ, 1e-7);
    EXPECT_NEAR(actual.bounds.MaxX, expected.bounds.MaxX, 1e-7);
    EXPECT_NEAR(actual.bounds.MaxY, expected.bounds.MaxY, 1e-7);
    EXPECT_NEAR(actual.bounds.MaxZ, expected.bounds.MaxZ, 1e-7);
}

PartDesign::Chamfer* expectParametricHistory(App::Document& document)
{
    EXPECT_EQ(document.getObjects().size(), 13);

    auto* body = dynamic_cast<PartDesign::Body*>(document.getObject("Body"));
    auto* pad = dynamic_cast<PartDesign::Pad*>(document.getObject("Pad"));
    auto* revolution =
        dynamic_cast<PartDesign::Revolution*>(document.getObject("Revolution"));
    auto* chamfer = dynamic_cast<PartDesign::Chamfer*>(document.getObject("Chamfer"));
    EXPECT_NE(body, nullptr);
    EXPECT_NE(pad, nullptr);
    EXPECT_NE(revolution, nullptr);
    EXPECT_NE(chamfer, nullptr);
    if (!body || !pad || !revolution || !chamfer) {
        return nullptr;
    }

    std::vector<std::string> history;
    for (const auto* object : body->getFullModel()) {
        history.emplace_back(object->getNameInDocument());
    }
    EXPECT_EQ(
        history,
        (std::vector<std::string> {"Sketch", "Pad", "Sketch001", "Revolution", "Chamfer"})
    );
    EXPECT_EQ(body->Tip.getValue(), chamfer);
    EXPECT_EQ(pad->Profile.getValue(), document.getObject("Sketch"));
    EXPECT_EQ(revolution->Profile.getValue(), document.getObject("Sketch001"));
    EXPECT_EQ(chamfer->Base.getValue(), revolution);
    EXPECT_EQ(chamfer->Base.getSubValues().size(), 4);

    return chamfer;
}

}  // namespace

class BackwardCompatibilityTest: public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        tests::initApplication();
        Base::Interpreter().runString("import PartDesign");
    }

    void SetUp() override
    {
        _testPath = App::Application::getHomePath() + "tests/TestModels/";
    }

    void TearDown() override
    {
        if (_doc) {
            App::GetApplication().closeDocument(_doc->getName());
        }
        if (!_temporaryFile.empty()) {
            std::remove(_temporaryFile.c_str());
        }
    }

    const App::Document* getDocument() const
    {
        return _doc;
    }

    void setDocument(App::Document* doc)
    {
        _doc = doc;
    }

    std::string getTestPath() const
    {
        return _testPath;
    }

    const std::string& setTemporaryFile(std::string path)
    {
        _temporaryFile = std::move(path);
        return _temporaryFile;
    }

private:
    App::Document* _doc = nullptr;
    std::string _testPath;
    std::string _temporaryFile;
};

TEST_F(BackwardCompatibilityTest, TestV021MigrationContractLifecycle)
{
    MigrationLogger logger;
    ScopedConsoleObserver observer(logger);
    auto* doc = App::GetApplication().openDocument(
        std::string(getTestPath() + "ModelFromV021.FCStd").c_str()
    );
    setDocument(doc);

    ASSERT_NE(doc, nullptr);
    EXPECT_EQ(std::string(doc->getProgramVersion()).find("0.21"), 0);
    auto* chamfer = expectParametricHistory(*doc);
    ASSERT_NE(chamfer, nullptr);
    const auto originalGeometry = geometryFingerprint(*chamfer);
    EXPECT_GT(originalGeometry.volume, 0.0);
    EXPECT_GT(originalGeometry.topology[1], 0);

    EXPECT_TRUE(std::ranges::any_of(logger.warnings, [](const std::string& warning) {
        return warning.find("being adjusted to maintain the same geometry") != std::string::npos
            && warning.find("FreeCAD 0.21.x") != std::string::npos;
    }));

    doc->recompute();
    ASSERT_FALSE(chamfer->isError());
    expectSameGeometry(originalGeometry, *chamfer);
    ASSERT_NE(expectParametricHistory(*doc), nullptr);

    const std::string originalPath = doc->getFileName();
    const auto& copyPath =
        setTemporaryFile(App::Application::getTempFileName() + std::string(".FCStd"));
    ASSERT_TRUE(doc->saveCopy(copyPath.c_str()));
    EXPECT_EQ(std::string(doc->getFileName()), originalPath);

    App::GetApplication().closeDocument(doc->getName());
    doc = App::GetApplication().openDocument(copyPath.c_str());
    setDocument(doc);
    ASSERT_NE(doc, nullptr);
    chamfer = expectParametricHistory(*doc);
    ASSERT_NE(chamfer, nullptr);
    expectSameGeometry(originalGeometry, *chamfer);
    doc->recompute();
    ASSERT_FALSE(chamfer->isError());
    expectSameGeometry(originalGeometry, *chamfer);
}

TEST_F(BackwardCompatibilityTest, TestTwoLengthsPadWithExpression)
{
    // Regression test for https://github.com/FreeCAD/FreeCAD/issues/28690 -- a v1.0.2 file with a
    // TwoLengths pad where Length has an expression (=10mm) and Length2 is a plain value (5mm). The
    // migration must swap both the values and the expressions so the geometry is preserved.

    auto doc = App::GetApplication().openDocument(
        std::string(getTestPath() + "TwoLengthsPadWithExpression.FCStd").c_str()
    );
    setDocument(doc);

    auto pad = dynamic_cast<PartDesign::Pad*>(doc->getObject("Pad"));
    ASSERT_NE(pad, nullptr);

    EXPECT_FALSE(pad->UseLegacyTaperDirection.getValue());
    EXPECT_DOUBLE_EQ(pad->Length.getValue(), 5.0);
    EXPECT_DOUBLE_EQ(pad->Length2.getValue(), 10.0);

    App::ObjectIdentifier lengthPath(pad->Length);
    App::ObjectIdentifier length2Path(pad->Length2);
    auto exprLength = pad->getExpression(lengthPath);
    auto exprLength2 = pad->getExpression(length2Path);
    EXPECT_FALSE(exprLength.expression);
    EXPECT_TRUE(exprLength2.expression);

    doc->recompute();
    auto bbox = pad->Shape.getBoundingBox();
    EXPECT_NEAR(bbox.MaxZ, 10.0, 0.01);
    EXPECT_NEAR(bbox.MinZ, -5.0, 0.01);
}

TEST_F(BackwardCompatibilityTest, TestTwoLengthsPadCyclicExpression)
{
    // Regression test for https://github.com/FreeCAD/FreeCAD/issues/29233 -- a v1.0 file where
    // Length2 has expression "Length / 2". Without the fix this becomes cyclic on migrate and is
    // silently discarded.

    auto doc = App::GetApplication().openDocument(
        std::string(getTestPath() + "TwoLengthsPadCyclicExpr.FCStd").c_str()
    );
    setDocument(doc);

    auto pad = dynamic_cast<PartDesign::Pad*>(doc->getObject("Pad"));
    ASSERT_NE(pad, nullptr);

    EXPECT_DOUBLE_EQ(pad->Length.getValue(), 5.0);
    EXPECT_DOUBLE_EQ(pad->Length2.getValue(), 10.0);

    App::ObjectIdentifier lengthPath(pad->Length);
    App::ObjectIdentifier length2Path(pad->Length2);
    auto exprLength = pad->getExpression(lengthPath);
    auto exprLength2 = pad->getExpression(length2Path);
    ASSERT_TRUE(exprLength.expression);  // "Length2 / 2" -- must not be discarded
    // The original expression was "Length / 2" on Length2. After renaming Length<->Length2,
    // it becomes "Length2 / 2" on Length — which is correct and not cyclic.
    std::string exprStr = exprLength.expression->toString();
    EXPECT_NE(exprStr.find("Length2"), std::string::npos)
        << "Expression on Length should reference Length2 after rename: " << exprStr;
    EXPECT_FALSE(exprLength2.expression);
}

TEST_F(BackwardCompatibilityTest, TestTwoLengthsPadCrossObjectRefExpression)
{
    // Regression test for https://github.com/FreeCAD/FreeCAD/issues/29233 -- a v1.0 file where
    // Sketch001.AttachmentOffset references Pad.Length2. After migration the reference must be
    // renamed to Pad.Length so it still points to the correct value.

    auto doc = App::GetApplication().openDocument(
        std::string(getTestPath() + "TwoLengthsPadCrossObjectRef.FCStd").c_str()
    );
    setDocument(doc);

    auto pad = dynamic_cast<PartDesign::Pad*>(doc->getObject("Pad"));
    ASSERT_NE(pad, nullptr);

    EXPECT_DOUBLE_EQ(pad->Length.getValue(), 8.0);
    EXPECT_DOUBLE_EQ(pad->Length2.getValue(), 10.0);

    auto sketch = doc->getObject("Sketch001");
    ASSERT_NE(sketch, nullptr);

    // AttachmentOffset.Base.z must reference Pad.Length (= -8mm), not Pad.Length2 (= -10mm).
    auto exprs = sketch->ExpressionEngine.getExpressions();
    const App::Expression* attachExpr = nullptr;
    for (const auto& [path, expr] : exprs) {
        if (path.toString().find("AttachmentOffset") != std::string::npos) {
            attachExpr = expr;
            break;
        }
    }
    ASSERT_NE(attachExpr, nullptr);
    std::string exprStr = attachExpr->toString();
    EXPECT_EQ(exprStr.find("Length2"), std::string::npos)
        << "Expression still references Length2 after migration: " << exprStr;
    EXPECT_NE(exprStr.find("Length"), std::string::npos)
        << "Expression does not reference Length after migration: " << exprStr;

    doc->recompute();
    auto* placement = dynamic_cast<App::PropertyPlacement*>(
        sketch->getPropertyByName("AttachmentOffset")
    );
    ASSERT_NE(placement, nullptr);
    EXPECT_NEAR(placement->getValue().getPosition().z, -8.0, 0.01);
}

// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
