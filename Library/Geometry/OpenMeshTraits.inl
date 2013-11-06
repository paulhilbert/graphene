template <>
inline bool OpenMeshTraits<TriMesh>::loadFromFile(MeshType& mesh, const std::string& path) {
	bool success = OpenMesh::IO::read_mesh(mesh, path);
	if (success) mesh.triangulate();
	return success;
}

template <class OpenMeshType>
inline bool OpenMeshTraits<OpenMeshType>::loadFromFile(MeshType& mesh, const std::string& path) {
	return OpenMesh::IO::read_mesh(mesh, path);
}

template <class OpenMeshType>
inline bool OpenMeshTraits<OpenMeshType>::saveToFile(const MeshType& mesh, const std::string& path) {
	return OpenMesh::IO::write_mesh(mesh, path);
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::Size OpenMeshTraits<OpenMeshType>::numVertices(const MeshType& mesh) {
	return mesh.n_vertices();
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::Size OpenMeshTraits<OpenMeshType>::numFaces(const MeshType& mesh) {
	return mesh.n_faces();
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::VertexId> OpenMeshTraits<OpenMeshType>::vertices(const MeshType& mesh) {
	std::vector<VertexId> vertices(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) vertices[i++] = it.handle().idx();
	return vertices;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::FaceId> OpenMeshTraits<OpenMeshType>::faces(const MeshType& mesh) {
	std::vector<FaceId> faces;
	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) faces.push_back(it.handle().idx());
	return faces;
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::PositionType OpenMeshTraits<OpenMeshType>::vertexPosition(const MeshType& mesh, VertexId id) {
	return Eigen::Vector3f(mesh.point(mesh.vertex_handle(id)).data());
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::NormalType OpenMeshTraits<OpenMeshType>::vertexNormal(const MeshType& mesh, VertexId id) {
	return Eigen::Vector3f(mesh.normal(mesh.vertex_handle(id)).data());
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::ColorType OpenMeshTraits<OpenMeshType>::vertexColor(const MeshType& mesh, VertexId id) {
	return Eigen::Vector4f(mesh.color(mesh.vertex_handle(id)).data());
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::NormalType OpenMeshTraits<OpenMeshType>::faceNormal(const MeshType& mesh, FaceId id) {
	return Eigen::Vector3f(mesh.normal(mesh.face_handle(id)).data());
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::PositionType> OpenMeshTraits<OpenMeshType>::vertexPositions(const MeshType& mesh) {
	std::vector<PositionType> positions(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) positions[i++] = mesh.point(it.handle());
	return positions;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::NormalType> OpenMeshTraits<OpenMeshType>::vertexNormals(const MeshType& mesh) {
	std::vector<PositionType> normals(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) normals[i++] = mesh.normal(it.handle());
	return normals;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::ColorType> OpenMeshTraits<OpenMeshType>::vertexColors(const MeshType& mesh) {
	std::vector<PositionType> colors(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) colors[i++] = mesh.color(it.handle());
	return colors;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::NormalType> OpenMeshTraits<OpenMeshType>::faceNormals(const MeshType& mesh) {
	std::vector<PositionType> normals(mesh.n_faces());
	unsigned int i=0;
	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) normals[i++] = mesh.normal(it.handle());
	return normals;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::VertexId> OpenMeshTraits<OpenMeshType>::faceVertices(const MeshType& mesh, FaceId face) {
	auto faceHandle = mesh.face_handle(face);
	std::vector<VertexId> vertices;
	for (auto fvIt = mesh.cfv_iter(faceHandle); fvIt; ++fvIt) {
		vertices.push_back(static_cast<VertexId>(fvIt.handle().idx()));
	}
	return vertices;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::FaceId> OpenMeshTraits<OpenMeshType>::adjacentFaces(const MeshType& mesh, FaceId face) {
	const auto& fHandle = mesh.face_handle(face);
	std::vector<FaceId> adj;
	for (auto ffIt=mesh.cff_iter(fHandle); ffIt; ++ffIt) {
		adj.push_back(ffIt.handle().idx());
	}
	return adj;
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::ScalarType OpenMeshTraits<OpenMeshType>::norm(const PositionType& p) {
	return p.norm();
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::PositionType OpenMeshTraits<OpenMeshType>::crossP(const PositionType& p0, const PositionType& p1) {
	return p0.cross(p1);
}
