template <class ColorType>
Mesh<ColorType>::Mesh(MeshPtrT mesh, bool smoothNormals) : m_mesh(mesh), m_smooth(smoothNormals) {
    m_vaoShadow = std::make_shared<harmont::vertex_array>();
    m_vaoDisplay = std::make_shared<harmont::vertex_array>();

    Eigen::MatrixXf vboData, posData;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> iboData;
    harmont::mesh_traits<MeshT>::buffer_data(*m_mesh, {harmont::POSITION, harmont::COLOR, harmont::NORMAL}, vboData, iboData, m_smooth);
    posData = vboData.block(0, 0, vboData.rows(), 3);
    m_numElements = iboData.rows();

    m_vboShadow = harmont::vertex_buffer<float>::from_data(posData);
    m_vboDisplay = harmont::vertex_buffer<float>::from_data(vboData);
    m_ibo = harmont::index_buffer<uint32_t>::from_data(iboData);
}

template <class ColorType>
Mesh<ColorType>::~Mesh() {
}

template <class ColorType>
void Mesh<ColorType>::init(harmont::shader_program::ptr program, harmont::pass_type_t type) {
    harmont::vertex_buffer<float>::layout_t vboLayout;
    if (type == harmont::SHADOW_GEOMETRY) {
        m_vaoShadow->bind();
        vboLayout = {{"position", 3}};
        m_vboShadow->bind_to_array(vboLayout, program);
        m_vaoShadow->release();
    } else {
        m_vaoDisplay->bind();
        vboLayout = {{"position", 3}, {"color", 1}, {"normal", 3}};
        m_vboDisplay->bind_to_array(vboLayout, program);
        m_vaoDisplay->release();
    }
}

template <class ColorType>
void Mesh<ColorType>::render(harmont::shader_program::ptr program, harmont::pass_type_t type) {
    if (type == harmont::SHADOW_GEOMETRY) {
        m_vaoShadow->bind();
    } else {
        m_vaoDisplay->bind();
    }
    m_ibo->bind();
    glDrawElements(GL_TRIANGLES, m_numElements, GL_UNSIGNED_INT, nullptr);
    if (type == harmont::SHADOW_GEOMETRY) {
        m_vaoShadow->release();
    } else {
        m_vaoDisplay->release();
    }
}
