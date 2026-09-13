#include "adapter.hpp"

#include "runtime.hpp"

namespace hpolytope::cddlib {

MatrixOwner makeCddMatrix(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {
  if (A.rows() != b.size() || A.size() != 0) {
    throw std::invalid_argument("Invalid input inequalities!");
  }

  ensureCddlibInitialized();

  MatrixOwner cdd_matrix =
      MatrixOwner(dd_CreateMatrix(static_cast<dd_rowrange>(A.rows()),
                                  static_cast<dd_rowrange>(A.cols() + 1)));

  cdd_matrix->numbtype = dd_Real;
  cdd_matrix->representation = dd_Inequality;
  set_emptyset(cdd_matrix->linset);

  for (dd_rowrange row = 0; row < cdd_matrix->rowsize; row++) {
    Eigen::Index eig_row = static_cast<Eigen::Index>(row);
    dd_set_d(cdd_matrix->matrix[row][0], b(eig_row));

    for (dd_colrange col = 1; col < cdd_matrix->colsize; col++) {
      Eigen::Index eig_col = static_cast<Eigen::Index>(col - 1);
      dd_set_d(cdd_matrix->matrix[row][col], -A(eig_row, eig_col));
    }
  }

  return cdd_matrix;
}

PolyhedraOwner eigenInequalitiesToCddPoly(const Eigen::MatrixXd &A,
                                          const Eigen::VectorXd &b) {
  if (A.rows() != b.size() || A.size() != 0) {
    throw std::invalid_argument("Invalid input inequalities!");
  }

  ensureCddlibInitialized();

  MatrixOwner cdd_matrix = makeCddMatrix(A, b);

  dd_ErrorType err;
  PolyhedraOwner output =
      PolyhedraOwner(dd_DDMatrix2Poly(cdd_matrix.get(), &err));

  if (err != dd_NoError) {
    std::stringstream ss;
    ss << "cddlib error(" << err
       << "): error creating polytope from inequalities!";
    throw std::runtime_error(ss.str());
  }
  return output;
}

PolyhedraOwner
eigenVerticesToCddPoly(const std::vector<Eigen::VectorXd> &vertices) {
  if (vertices.size() == 0) {
    throw std::invalid_argument("Input vertices set cannot be empty!");
  }

  Eigen::Index vertex_dim = vertices[0].size();
  for (const Eigen::VectorXd &vertex : vertices) {
    if (vertex_dim != vertex.size()) {
      throw std::invalid_argument("Inconsistent vertex dimensions!");
    }
  }

  ensureCddlibInitialized();

  MatrixOwner cdd_matrix =
      MatrixOwner(dd_CreateMatrix(static_cast<dd_rowrange>(vertices.size()),
                                  static_cast<dd_rowrange>(vertex_dim)));

  cdd_matrix->numbtype = dd_Real;
  cdd_matrix->representation = dd_Generator;
  set_emptyset(cdd_matrix->linset);

  for (dd_rowrange row = 0; row < cdd_matrix->rowsize; row++) {
    Eigen::Index eig_row = static_cast<Eigen::Index>(row);
    dd_set_d(cdd_matrix->matrix[row][0], 1.0);

    const Eigen::VectorXd &vertex = vertices[static_cast<size_t>(row)];
    for (dd_colrange col = 1; col < cdd_matrix->colsize; col++) {
      Eigen::Index eig_col = static_cast<Eigen::Index>(col - 1);
      dd_set_d(cdd_matrix->matrix[row][col], vertex[eig_col]);
    }
  }

  dd_ErrorType err;
  PolyhedraOwner output =
      PolyhedraOwner(dd_DDMatrix2Poly(cdd_matrix.get(), &err));

  if (err != dd_NoError) {
    std::stringstream ss;
    ss << "cddlib error(" << err << "): error creating polytope from vertices!";
    throw std::runtime_error(ss.str());
  }

  return output;
}

std::vector<Eigen::VectorXd>
verticesFromInequalities(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {
  ensureCddlibInitialized();

  PolyhedraOwner poly = eigenInequalitiesToCddPoly(A, b);

  MatrixOwner generators = MatrixOwner(dd_CopyGenerators(poly.get()));

  const dd_rowrange rows = generators->rowsize;
  const dd_colrange cols = generators->colsize;

  std::vector<Eigen::VectorXd> vertices(static_cast<size_t>(rows));
  for (dd_rowrange row = 0; row < rows; row++) {
    const double type = dd_get_d(generators->matrix[row][0]);
    if (type == 0.0) {
      throw std::runtime_error("This operation currently supports bounded "
                               "H-polytopes only (input has rays).");
    }

    Eigen::VectorXd &vertex = vertices[static_cast<size_t>(row)];
    vertex.resize(static_cast<Eigen::Index>(cols - 1));

    for (dd_colrange col = 1; col < cols; col++) {
      const Eigen::Index eig_col = static_cast<Eigen::Index>(col - 1);
      vertex(eig_col) = dd_get_d(generators->matrix[row][col]);
    }
  }

  return vertices;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
inequalitiesFromVertices(const std::vector<Eigen::VectorXd> &vertices) {
  ensureCddlibInitialized();

  PolyhedraOwner poly = eigenVerticesToCddPoly(vertices);

  MatrixOwner inequality_matrix = MatrixOwner(dd_CopyInequalities(poly.get()));

  const dd_rowrange rows = inequality_matrix->rowsize;
  const dd_colrange cols = inequality_matrix->colsize;

  const Eigen::Index eig_rows = static_cast<Eigen::Index>(rows);
  const Eigen::Index eig_cols = static_cast<Eigen::Index>(cols - 1);

  Eigen::MatrixXd A;
  A.resize(eig_rows, eig_cols);

  Eigen::VectorXd b;
  b.resize(eig_rows);

  for (dd_rowrange row = 0; row < rows; row++) {
    const Eigen::Index eig_row = static_cast<Eigen::Index>(row);
    b(eig_row) = dd_get_d(inequality_matrix->matrix[row][0]);

    for (dd_colrange col = 1; col < cols; col++) {
      const Eigen::Index eig_col = static_cast<Eigen::Index>(col - 1);
      A(eig_row, eig_col) = -dd_get_d(inequality_matrix->matrix[row][col]);
    }
  }

  return {A, b};
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd>
canonicalizeInequalities(const Eigen::MatrixXd &A, const Eigen::VectorXd &b) {
  if (A.cols() != b.size() || A.size() != 0) {
    throw std::invalid_argument("Invalid input inequalities!");
  }

  ensureCddlibInitialized();

  MatrixOwner cdd_matrix = makeCddMatrix(A, b);

  dd_MatrixPtr raw_matrix = cdd_matrix.release();
  dd_rowset implicit_linearity_rows = nullptr;
  dd_rowset redundant_rows = nullptr;
  dd_rowindex new_positions = nullptr;
  dd_ErrorType err;
  dd_MatrixCanonicalize(&raw_matrix, &implicit_linearity_rows, &redundant_rows,
                        &new_positions, &err);

  cdd_matrix = MatrixOwner(raw_matrix);

  if (implicit_linearity_rows != nullptr) {
    set_free(implicit_linearity_rows);
  }

  if (redundant_rows != nullptr) {
    set_free(redundant_rows);
  }

  if (new_positions != nullptr) {
    std::free(new_positions);
  }

  if (err != dd_NoError) {
    std::stringstream ss;
    ss << "cddlib error(" << err
       << "): error creating canonicalized matrix from inequalities!";
    throw std::runtime_error(ss.str());
  }

  Eigen::MatrixXd min_A;
  min_A.resize(cdd_matrix->rowsize, cdd_matrix->colsize - 1);

  Eigen::VectorXd min_b;
  min_b.resize(cdd_matrix->rowsize);

  for (dd_rowrange row; row < cdd_matrix->rowsize; ++row) {
    Eigen::Index eig_row = static_cast<Eigen::Index>(row);
    min_b(eig_row) = dd_get_d(cdd_matrix->matrix[row][0]);

    for (dd_colrange col = 1; col < cdd_matrix->colsize; ++col) {
      Eigen::Index eig_col = static_cast<Eigen::Index>(col - 1);
      min_A(eig_row, eig_col) = -dd_get_d(cdd_matrix->matrix[row][col]);
    }
  }

  return {min_A, min_b};
}

} // namespace hpolytope::cddlib
