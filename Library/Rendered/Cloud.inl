template <class PointT>
Cloud<PointT>::Cloud(typename CloudT::Ptr cloud, Eigen::Vector4f default_color) : m_cloud(cloud) {
    m_vaoShadow = std::make_shared<harmont::vertex_array>();
    m_vaoDisplay = std::make_shared<harmont::vertex_array>();

    Eigen::MatrixXf vboData, posData;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> iboData;
    harmont::pointcloud_traits<CloudT>::buffer_data(*m_cloud, {harmont::POSITION, harmont::COLOR, harmont::NORMAL}, vboData, iboData, default_color);
    posData = vboData.block(0, 0, vboData.rows(), 3);
    m_numElements = iboData.rows();

    m_vboShadow = harmont::vertex_buffer<float>::from_data(posData);
    m_vboDisplay = harmont::vertex_buffer<float>::from_data(vboData);
    m_ibo = harmont::index_buffer<uint32_t>::from_data(iboData);
}

template <class PointT>
Cloud<PointT>::~Cloud() {
}

template <class PointT>
void Cloud<PointT>::init(harmont::shader_program::ptr program, harmont::pass_type_t type) {
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

template <class PointT>
void Cloud<PointT>::render(harmont::shader_program::ptr program, harmont::pass_type_t type) {
    if (type == harmont::SHADOW_GEOMETRY) {
        m_vaoShadow->bind();
    } else {
        m_vaoDisplay->bind();
    }
    m_ibo->bind();
    glDrawElements(GL_POINTS, m_numElements, GL_UNSIGNED_INT, nullptr);
    if (type == harmont::SHADOW_GEOMETRY) {
        m_vaoShadow->release();
    } else {
        m_vaoDisplay->release();
    }
}
