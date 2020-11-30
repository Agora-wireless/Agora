/*
 * Util functions specific to channel simulator 
 *
 *
 *
 */

#define NUM_ROWS_PRINT 10   // TODO: Remove in the future. Used to truncate printout

static inline void print_cxmat(cx_fmat c)
{
    size_t print_rows = std::min((size_t)c.n_rows, (size_t)NUM_ROWS_PRINT);
    std::cout << "Printing only " << print_rows << " rows !!" << std::endl;
    std::stringstream so;
    for (size_t i = 0; i < c.n_cols; i++) {
        so << "col" << i << " = [";
        for (size_t j = 0; j < print_rows; j++)
            so << std::fixed << std::setw(8) << std::setprecision(6)
               << c.at(j, i).real() << "+" << c.at(j, i).imag() << "i ";
        so << "];\n";
    }
    so << std::endl;
    std::cout << so.str();
}

static inline void print_cxvec(cx_frowvec c)
{
    std::stringstream so;
    for (size_t i = 0; i < c.n_elem; i++) {
        so << "vec" << i << " = [";
        so << std::fixed << std::setw(8) << std::setprecision(6)
	   << c.at(i).real() << "+" << c.at(i).imag() << "i ";
        so << "];\n";
    }
    so << std::endl;
    std::cout << so.str();
}

static inline void print_mat(fmat c)
{
    size_t print_rows = std::min((size_t)c.n_rows, (size_t)NUM_ROWS_PRINT);
    std::cout << "Printing only " << print_rows << " rows !!" << std::endl;
    std::stringstream so;
    for (size_t i = 0; i < c.n_cols; i++) {
        so << "col" << i << " = [";
        for (size_t j = 0; j < print_rows; j++)
            so << std::fixed << std::setw(8) << std::setprecision(6)
               << c.at(j, i);
        so << "];\n";
    }
    so << std::endl;
    std::cout << so.str();
}

static inline void print_vec(frowvec c)
{
    std::stringstream so;
    for (size_t i = 0; i < c.n_elem; i++) {
        so << "vec" << i << " = [";
        so << std::fixed << std::setw(8) << std::setprecision(6)
           << c.at(i);
        so << "];\n";
    }
    so << std::endl;
    std::cout << so.str();
}
