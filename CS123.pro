# -------------------------------------------------
# Project created by QtCreator 2010-08-22T14:12:19
# -------------------------------------------------
QT += opengl xml
TARGET = CS123
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++14
CONFIG += c++14

QMAKE_LFLAGS += -stdlib=libc++
QMAKE_CXXFLAGS += -stdlib=libc++

unix:!macx {
    LIBS += -lGLU
}
macx {
    QMAKE_CFLAGS_X86_64 += -mmacosx-version-min=10.7
    QMAKE_CXXFLAGS_X86_64 = $$QMAKE_CFLAGS_X86_64
    CONFIG += c++11
}
win32 {
    DEFINES += GLEW_STATIC
    LIBS += -lopengl32 -lglu32
}

SOURCES += \
    scenegraph/Scene.cpp \
    scenegraph/OpenGLScene.cpp \
    scenegraph/RayScene.cpp \
    ui/Canvas2D.cpp \
    ui/SupportCanvas2D.cpp \
    ui/Settings.cpp \
    ui/mainwindow.cpp \
    ui/Databinding.cpp \
    lib/BGRA.cpp \
    lib/CS123XmlSceneParser.cpp \
    lib/ResourceLoader.cpp \
    gl/shaders/Shader.cpp \
    gl/GLDebug.cpp \
    gl/datatype/VBOAttribMarker.cpp \
    gl/datatype/VBO.cpp \
    gl/datatype/IBO.cpp \
    gl/datatype/VAO.cpp \
    gl/datatype/FBO.cpp \
    gl/textures/Texture.cpp \
    gl/textures/Texture2D.cpp \
    gl/textures/TextureParameters.cpp \
    gl/textures/TextureParametersBuilder.cpp \
    gl/textures/RenderBuffer.cpp \
    gl/textures/DepthBuffer.cpp \
    gl/shaders/CS123Shader.cpp \
    gl/util/FullScreenQuad.cpp \
    main.cpp \
    glew-1.10.0/src/glew.c \
    materials/bilateralfilter.cpp \
    materials/brdfreplacement.cpp \
    materials/causticmaker.cpp \
    materials/histogram.cpp \
    materials/imagereader.cpp \
    materials/incidentlight.cpp \
    materials/loggabor.cpp \
    materials/materialmanager.cpp \
    materials/retexture.cpp \
    materials/shapeestimation.cpp


HEADERS += \
    scenegraph/Scene.h \
    scenegraph/OpenGLScene.h \
    scenegraph/RayScene.h \
    ui/Canvas2D.h \
    ui/SupportCanvas2D.h \
    ui/Settings.h \
    ui/mainwindow.h \
    ui/Databinding.h \
    ui_mainwindow.h \
    gl/shaders/Shader.h \
    gl/GLDebug.h \
    gl/shaders/ShaderAttribLocations.h \
    gl/datatype/VBOAttribMarker.h \
    gl/datatype/VBO.h \
    gl/datatype/IBO.h \
    gl/datatype/VAO.h \
    gl/datatype/FBO.h \
    gl/textures/Texture.h \
    gl/textures/Texture2D.h \
    gl/textures/TextureParameters.h \
    gl/textures/TextureParametersBuilder.h \
    gl/textures/RenderBuffer.h \
    gl/textures/DepthBuffer.h \
    gl/shaders/CS123Shader.h \
    gl/util/FullScreenQuad.h \
    lib/BGRA.h \
    lib/CS123XmlSceneParser.h \
    lib/CS123SceneData.h \
    lib/CS123ISceneParser.h \
    lib/ResourceLoader.h \
    glew-1.10.0/include/GL/glew.h \
    Eigen/src/Cholesky/LDLT.h \
    Eigen/src/Cholesky/LLT.h \
    Eigen/src/Cholesky/LLT_LAPACKE.h \
    Eigen/src/CholmodSupport/CholmodSupport.h \
    Eigen/src/Core/arch/AltiVec/Complex.h \
    Eigen/src/Core/arch/AltiVec/MathFunctions.h \
    Eigen/src/Core/arch/AltiVec/PacketMath.h \
    Eigen/src/Core/arch/AVX/Complex.h \
    Eigen/src/Core/arch/AVX/MathFunctions.h \
    Eigen/src/Core/arch/AVX/PacketMath.h \
    Eigen/src/Core/arch/AVX/TypeCasting.h \
    Eigen/src/Core/arch/AVX512/MathFunctions.h \
    Eigen/src/Core/arch/AVX512/PacketMath.h \
    Eigen/src/Core/arch/CUDA/Complex.h \
    Eigen/src/Core/arch/CUDA/Half.h \
    Eigen/src/Core/arch/CUDA/MathFunctions.h \
    Eigen/src/Core/arch/CUDA/PacketMath.h \
    Eigen/src/Core/arch/CUDA/PacketMathHalf.h \
    Eigen/src/Core/arch/CUDA/TypeCasting.h \
    Eigen/src/Core/arch/Default/Settings.h \
    Eigen/src/Core/arch/NEON/Complex.h \
    Eigen/src/Core/arch/NEON/MathFunctions.h \
    Eigen/src/Core/arch/NEON/PacketMath.h \
    Eigen/src/Core/arch/SSE/Complex.h \
    Eigen/src/Core/arch/SSE/MathFunctions.h \
    Eigen/src/Core/arch/SSE/PacketMath.h \
    Eigen/src/Core/arch/SSE/TypeCasting.h \
    Eigen/src/Core/arch/ZVector/Complex.h \
    Eigen/src/Core/arch/ZVector/MathFunctions.h \
    Eigen/src/Core/arch/ZVector/PacketMath.h \
    Eigen/src/Core/functors/AssignmentFunctors.h \
    Eigen/src/Core/functors/BinaryFunctors.h \
    Eigen/src/Core/functors/NullaryFunctors.h \
    Eigen/src/Core/functors/StlFunctors.h \
    Eigen/src/Core/functors/TernaryFunctors.h \
    Eigen/src/Core/functors/UnaryFunctors.h \
    Eigen/src/Core/products/GeneralBlockPanelKernel.h \
    Eigen/src/Core/products/GeneralMatrixMatrix.h \
    Eigen/src/Core/products/GeneralMatrixMatrix_BLAS.h \
    Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
    Eigen/src/Core/products/GeneralMatrixMatrixTriangular_BLAS.h \
    Eigen/src/Core/products/GeneralMatrixVector.h \
    Eigen/src/Core/products/GeneralMatrixVector_BLAS.h \
    Eigen/src/Core/products/Parallelizer.h \
    Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
    Eigen/src/Core/products/SelfadjointMatrixMatrix_BLAS.h \
    Eigen/src/Core/products/SelfadjointMatrixVector.h \
    Eigen/src/Core/products/SelfadjointMatrixVector_BLAS.h \
    Eigen/src/Core/products/SelfadjointProduct.h \
    Eigen/src/Core/products/SelfadjointRank2Update.h \
    Eigen/src/Core/products/TriangularMatrixMatrix.h \
    Eigen/src/Core/products/TriangularMatrixMatrix_BLAS.h \
    Eigen/src/Core/products/TriangularMatrixVector.h \
    Eigen/src/Core/products/TriangularMatrixVector_BLAS.h \
    Eigen/src/Core/products/TriangularSolverMatrix.h \
    Eigen/src/Core/products/TriangularSolverMatrix_BLAS.h \
    Eigen/src/Core/products/TriangularSolverVector.h \
    Eigen/src/Core/util/BlasUtil.h \
    Eigen/src/Core/util/Constants.h \
    Eigen/src/Core/util/DisableStupidWarnings.h \
    Eigen/src/Core/util/ForwardDeclarations.h \
    Eigen/src/Core/util/Macros.h \
    Eigen/src/Core/util/Memory.h \
    Eigen/src/Core/util/Meta.h \
    Eigen/src/Core/util/MKL_support.h \
    Eigen/src/Core/util/NonMPL2.h \
    Eigen/src/Core/util/ReenableStupidWarnings.h \
    Eigen/src/Core/util/StaticAssert.h \
    Eigen/src/Core/util/XprHelper.h \
    Eigen/src/Core/Array.h \
    Eigen/src/Core/ArrayBase.h \
    Eigen/src/Core/ArrayWrapper.h \
    Eigen/src/Core/Assign.h \
    Eigen/src/Core/Assign_MKL.h \
    Eigen/src/Core/AssignEvaluator.h \
    Eigen/src/Core/BandMatrix.h \
    Eigen/src/Core/Block.h \
    Eigen/src/Core/BooleanRedux.h \
    Eigen/src/Core/CommaInitializer.h \
    Eigen/src/Core/ConditionEstimator.h \
    Eigen/src/Core/CoreEvaluators.h \
    Eigen/src/Core/CoreIterators.h \
    Eigen/src/Core/CwiseBinaryOp.h \
    Eigen/src/Core/CwiseNullaryOp.h \
    Eigen/src/Core/CwiseTernaryOp.h \
    Eigen/src/Core/CwiseUnaryOp.h \
    Eigen/src/Core/CwiseUnaryView.h \
    Eigen/src/Core/DenseBase.h \
    Eigen/src/Core/DenseCoeffsBase.h \
    Eigen/src/Core/DenseStorage.h \
    Eigen/src/Core/Diagonal.h \
    Eigen/src/Core/DiagonalMatrix.h \
    Eigen/src/Core/DiagonalProduct.h \
    Eigen/src/Core/Dot.h \
    Eigen/src/Core/EigenBase.h \
    Eigen/src/Core/ForceAlignedAccess.h \
    Eigen/src/Core/Fuzzy.h \
    Eigen/src/Core/GeneralProduct.h \
    Eigen/src/Core/GenericPacketMath.h \
    Eigen/src/Core/GlobalFunctions.h \
    Eigen/src/Core/Inverse.h \
    Eigen/src/Core/IO.h \
    Eigen/src/Core/Map.h \
    Eigen/src/Core/MapBase.h \
    Eigen/src/Core/MathFunctions.h \
    Eigen/src/Core/MathFunctionsImpl.h \
    Eigen/src/Core/Matrix.h \
    Eigen/src/Core/MatrixBase.h \
    Eigen/src/Core/NestByValue.h \
    Eigen/src/Core/NoAlias.h \
    Eigen/src/Core/NumTraits.h \
    Eigen/src/Core/PermutationMatrix.h \
    Eigen/src/Core/PlainObjectBase.h \
    Eigen/src/Core/Product.h \
    Eigen/src/Core/ProductEvaluators.h \
    Eigen/src/Core/Random.h \
    Eigen/src/Core/Redux.h \
    Eigen/src/Core/Ref.h \
    Eigen/src/Core/Replicate.h \
    Eigen/src/Core/ReturnByValue.h \
    Eigen/src/Core/Reverse.h \
    Eigen/src/Core/Select.h \
    Eigen/src/Core/SelfAdjointView.h \
    Eigen/src/Core/SelfCwiseBinaryOp.h \
    Eigen/src/Core/Solve.h \
    Eigen/src/Core/SolverBase.h \
    Eigen/src/Core/SolveTriangular.h \
    Eigen/src/Core/StableNorm.h \
    Eigen/src/Core/Stride.h \
    Eigen/src/Core/Swap.h \
    Eigen/src/Core/Transpose.h \
    Eigen/src/Core/Transpositions.h \
    Eigen/src/Core/TriangularMatrix.h \
    Eigen/src/Core/VectorBlock.h \
    Eigen/src/Core/VectorwiseOp.h \
    Eigen/src/Core/Visitor.h \
    Eigen/src/Eigenvalues/ComplexEigenSolver.h \
    Eigen/src/Eigenvalues/ComplexSchur.h \
    Eigen/src/Eigenvalues/ComplexSchur_LAPACKE.h \
    Eigen/src/Eigenvalues/EigenSolver.h \
    Eigen/src/Eigenvalues/GeneralizedEigenSolver.h \
    Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
    Eigen/src/Eigenvalues/HessenbergDecomposition.h \
    Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
    Eigen/src/Eigenvalues/RealQZ.h \
    Eigen/src/Eigenvalues/RealSchur.h \
    Eigen/src/Eigenvalues/RealSchur_LAPACKE.h \
    Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
    Eigen/src/Eigenvalues/SelfAdjointEigenSolver_LAPACKE.h \
    Eigen/src/Eigenvalues/Tridiagonalization.h \
    Eigen/src/Geometry/arch/Geometry_SSE.h \
    Eigen/src/Geometry/AlignedBox.h \
    Eigen/src/Geometry/AngleAxis.h \
    Eigen/src/Geometry/EulerAngles.h \
    Eigen/src/Geometry/Homogeneous.h \
    Eigen/src/Geometry/Hyperplane.h \
    Eigen/src/Geometry/OrthoMethods.h \
    Eigen/src/Geometry/ParametrizedLine.h \
    Eigen/src/Geometry/Quaternion.h \
    Eigen/src/Geometry/Rotation2D.h \
    Eigen/src/Geometry/RotationBase.h \
    Eigen/src/Geometry/Scaling.h \
    Eigen/src/Geometry/Transform.h \
    Eigen/src/Geometry/Translation.h \
    Eigen/src/Geometry/Umeyama.h \
    Eigen/src/Householder/BlockHouseholder.h \
    Eigen/src/Householder/Householder.h \
    Eigen/src/Householder/HouseholderSequence.h \
    Eigen/src/IterativeLinearSolvers/BasicPreconditioners.h \
    Eigen/src/IterativeLinearSolvers/BiCGSTAB.h \
    Eigen/src/IterativeLinearSolvers/ConjugateGradient.h \
    Eigen/src/IterativeLinearSolvers/IncompleteCholesky.h \
    Eigen/src/IterativeLinearSolvers/IncompleteLUT.h \
    Eigen/src/IterativeLinearSolvers/IterativeSolverBase.h \
    Eigen/src/IterativeLinearSolvers/LeastSquareConjugateGradient.h \
    Eigen/src/IterativeLinearSolvers/SolveWithGuess.h \
    Eigen/src/Jacobi/Jacobi.h \
    Eigen/src/LU/arch/Inverse_SSE.h \
    Eigen/src/LU/Determinant.h \
    Eigen/src/LU/FullPivLU.h \
    Eigen/src/LU/InverseImpl.h \
    Eigen/src/LU/PartialPivLU.h \
    Eigen/src/LU/PartialPivLU_LAPACKE.h \
    Eigen/src/MetisSupport/MetisSupport.h \
    Eigen/src/misc/blas.h \
    Eigen/src/misc/Image.h \
    Eigen/src/misc/Kernel.h \
    Eigen/src/misc/lapack.h \
    Eigen/src/misc/lapacke.h \
    Eigen/src/misc/lapacke_mangling.h \
    Eigen/src/misc/RealSvd2x2.h \
    Eigen/src/OrderingMethods/Amd.h \
    Eigen/src/OrderingMethods/Eigen_Colamd.h \
    Eigen/src/OrderingMethods/Ordering.h \
    Eigen/src/PardisoSupport/PardisoSupport.h \
    Eigen/src/PaStiXSupport/PaStiXSupport.h \
    Eigen/src/plugins/ArrayCwiseBinaryOps.h \
    Eigen/src/plugins/ArrayCwiseUnaryOps.h \
    Eigen/src/plugins/BlockMethods.h \
    Eigen/src/plugins/CommonCwiseBinaryOps.h \
    Eigen/src/plugins/CommonCwiseUnaryOps.h \
    Eigen/src/plugins/MatrixCwiseBinaryOps.h \
    Eigen/src/plugins/MatrixCwiseUnaryOps.h \
    Eigen/src/QR/ColPivHouseholderQR.h \
    Eigen/src/QR/ColPivHouseholderQR_LAPACKE.h \
    Eigen/src/QR/CompleteOrthogonalDecomposition.h \
    Eigen/src/QR/FullPivHouseholderQR.h \
    Eigen/src/QR/HouseholderQR.h \
    Eigen/src/QR/HouseholderQR_LAPACKE.h \
    Eigen/src/SparseCholesky/SimplicialCholesky.h \
    Eigen/src/SparseCholesky/SimplicialCholesky_impl.h \
    Eigen/src/SparseCore/AmbiVector.h \
    Eigen/src/SparseCore/CompressedStorage.h \
    Eigen/src/SparseCore/ConservativeSparseSparseProduct.h \
    Eigen/src/SparseCore/MappedSparseMatrix.h \
    Eigen/src/SparseCore/SparseAssign.h \
    Eigen/src/SparseCore/SparseBlock.h \
    Eigen/src/SparseCore/SparseColEtree.h \
    Eigen/src/SparseCore/SparseCompressedBase.h \
    Eigen/src/SparseCore/SparseCwiseBinaryOp.h \
    Eigen/src/SparseCore/SparseCwiseUnaryOp.h \
    Eigen/src/SparseCore/SparseDenseProduct.h \
    Eigen/src/SparseCore/SparseDiagonalProduct.h \
    Eigen/src/SparseCore/SparseDot.h \
    Eigen/src/SparseCore/SparseFuzzy.h \
    Eigen/src/SparseCore/SparseMap.h \
    Eigen/src/SparseCore/SparseMatrix.h \
    Eigen/src/SparseCore/SparseMatrixBase.h \
    Eigen/src/SparseCore/SparsePermutation.h \
    Eigen/src/SparseCore/SparseProduct.h \
    Eigen/src/SparseCore/SparseRedux.h \
    Eigen/src/SparseCore/SparseRef.h \
    Eigen/src/SparseCore/SparseSelfAdjointView.h \
    Eigen/src/SparseCore/SparseSolverBase.h \
    Eigen/src/SparseCore/SparseSparseProductWithPruning.h \
    Eigen/src/SparseCore/SparseTranspose.h \
    Eigen/src/SparseCore/SparseTriangularView.h \
    Eigen/src/SparseCore/SparseUtil.h \
    Eigen/src/SparseCore/SparseVector.h \
    Eigen/src/SparseCore/SparseView.h \
    Eigen/src/SparseCore/TriangularSolver.h \
    Eigen/src/SparseLU/SparseLU.h \
    Eigen/src/SparseLU/SparseLU_column_bmod.h \
    Eigen/src/SparseLU/SparseLU_column_dfs.h \
    Eigen/src/SparseLU/SparseLU_copy_to_ucol.h \
    Eigen/src/SparseLU/SparseLU_gemm_kernel.h \
    Eigen/src/SparseLU/SparseLU_heap_relax_snode.h \
    Eigen/src/SparseLU/SparseLU_kernel_bmod.h \
    Eigen/src/SparseLU/SparseLU_Memory.h \
    Eigen/src/SparseLU/SparseLU_panel_bmod.h \
    Eigen/src/SparseLU/SparseLU_panel_dfs.h \
    Eigen/src/SparseLU/SparseLU_pivotL.h \
    Eigen/src/SparseLU/SparseLU_pruneL.h \
    Eigen/src/SparseLU/SparseLU_relax_snode.h \
    Eigen/src/SparseLU/SparseLU_Structs.h \
    Eigen/src/SparseLU/SparseLU_SupernodalMatrix.h \
    Eigen/src/SparseLU/SparseLU_Utils.h \
    Eigen/src/SparseLU/SparseLUImpl.h \
    Eigen/src/SparseQR/SparseQR.h \
    Eigen/src/SPQRSupport/SuiteSparseQRSupport.h \
    Eigen/src/StlSupport/details.h \
    Eigen/src/StlSupport/StdDeque.h \
    Eigen/src/StlSupport/StdList.h \
    Eigen/src/StlSupport/StdVector.h \
    Eigen/src/SuperLUSupport/SuperLUSupport.h \
    Eigen/src/SVD/BDCSVD.h \
    Eigen/src/SVD/JacobiSVD.h \
    Eigen/src/SVD/JacobiSVD_LAPACKE.h \
    Eigen/src/SVD/SVDBase.h \
    Eigen/src/SVD/UpperBidiagonalization.h \
    Eigen/src/UmfPackSupport/UmfPackSupport.h \
    Eigen/Cholesky \
    Eigen/CholmodSupport \
    Eigen/Core \
    Eigen/Dense \
    Eigen/Eigen \
    Eigen/Eigenvalues \
    Eigen/Geometry \
    Eigen/Householder \
    Eigen/IterativeLinearSolvers \
    Eigen/Jacobi \
    Eigen/LU \
    Eigen/MetisSupport \
    Eigen/OrderingMethods \
    Eigen/PardisoSupport \
    Eigen/PaStiXSupport \
    Eigen/QR \
    Eigen/QtAlignedMalloc \
    Eigen/Sparse \
    Eigen/SparseCholesky \
    Eigen/SparseCore \
    Eigen/SparseLU \
    Eigen/SparseQR \
    Eigen/SPQRSupport \
    Eigen/StdDeque \
    Eigen/StdList \
    Eigen/StdVector \
    Eigen/SuperLUSupport \
    Eigen/SVD \
    Eigen/UmfPackSupport \
    materials/bilateralfilter.h \
    materials/brdfreplacement.h \
    materials/causticmaker.h \
    materials/histogram.h \
    materials/imagereader.h \
    materials/incidentlight.h \
    materials/loggabor.h \
    materials/materialmanager.h \
    materials/retexture.h \
    materials/shapeestimation.h \
    materials/ui_mainwindow.h


FORMS += ui/mainwindow.ui
INCLUDEPATH += glm brush camera lib scenegraph ui glew-1.10.0/include
DEPENDPATH += glm brush camera lib scenegraph ui glew-1.10.0/include
DEFINES += _USE_MATH_DEFINES
DEFINES += TIXML_USE_STL
DEFINES += GLM_SWIZZLE GLM_FORCE_RADIANS
OTHER_FILES += shaders/shader.frag \
    shaders/shader.vert \
    shaders/wireframe/wireframe.vert \
    shaders/wireframe/wireframe.frag \
    shaders/normals/normals.vert \
    shaders/normals/normals.gsh \
    shaders/normals/normals.frag \
    shaders/normals/normalsArrow.vert \
    shaders/normals/normalsArrow.gsh \
    shaders/normals/normalsArrow.frag \
    shaders/deferredlighting/gbuffer/gbuffer.frag \
    shaders/deferredlighting/gbuffer/gbuffer.vert \
    shaders/deferredlighting/lighting/lighting.frag \
    shaders/deferredlighting/lighting/lighting.vert \
    shaders/deferredlighting/compositing/compositing.frag \
    shaders/deferredlighting/compositing/compositing.vert

# Don't add the -pg flag unless you know what you are doing. It makes QThreadPool freeze on Mac OS X
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3
QMAKE_CXXFLAGS_WARN_ON -= -Wall
QMAKE_CXXFLAGS_WARN_ON += -Waddress -Warray-bounds -Wc++0x-compat -Wchar-subscripts -Wformat\
                          -Wmain -Wmissing-braces -Wparentheses -Wreorder -Wreturn-type \
                          -Wsequence-point -Wsign-compare -Wstrict-overflow=1 -Wswitch \
                          -Wtrigraphs -Wuninitialized -Wunused-label -Wunused-variable \
                          -Wvolatile-register-var -Wno-extra

QMAKE_CXXFLAGS += -g

# QMAKE_CXX_FLAGS_WARN_ON += -Wunknown-pragmas -Wunused-function -Wmain

macx {
    QMAKE_CXXFLAGS_WARN_ON -= -Warray-bounds -Wc++0x-compat
}

RESOURCES += \
    resources.qrc

DISTFILES += \
    shaders/normals/normals.vert \
    shaders/normals/normals.frag \
    shaders/normals/normals.gsh \
    shaders/normals/normalsArrow.gsh \
    shaders/normals/normalsArrow.frag \
    shaders/normals/normalsArrow.vert \
    Eigen/CMakeLists.txt \
    materials/target_caustic_inverse \
    materials/brdfsolver.py \
    materials/caustic_finder.py \
    materials/design_caustic.py \
    materials/maskbuilder.py \
    materials/phaseSymmetry.py \
    materials/target_caustic.py \
    materials/target_caustic_cracks.py \
    materials/target_caustic_cracks_inverse.py \
    materials/target_caustic_inverse.py \
    materials/target_caustic_spheres.py \
    materials/working_han.py
