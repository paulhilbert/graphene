template <>
inline bool OpenMeshTraits<TriMesh>::loadFromFile(MeshType& mesh, const std::string& path) {
	// The options define which properties we would like to have loaded.
	// After the read_mesh call, "opt" will contain the attributes which
	// were actually present in the file.
	OpenMesh::IO::Options opt;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexColor;
	
	bool success = OpenMesh::IO::read_mesh(mesh, path, opt);
	
	if (success) {
		mesh.triangulate();
		// If no face normals were loaded, estimate them.
		if (!opt.face_has_normal()) {
			mesh.update_face_normals();
		}
		// If no vertex normals were loaded, estimate them.
		// Note that OpenMesh requires face normals to be available for this.
		if (!opt.vertex_has_normal()) {
			mesh.update_normals();
		}
		// If no vertex colors were loaded, set a default value for all vertices.
		if (!opt.vertex_has_color()) {
			for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
				mesh.set_color(it.handle(), InternalOpenMeshTraits::Color(1,1,1,1));
			}
		}
	}
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
	return PositionType(mesh.point(mesh.vertex_handle(id)).data());
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::NormalType OpenMeshTraits<OpenMeshType>::vertexNormal(const MeshType& mesh, VertexId id) {
	return NormalType(mesh.normal(mesh.vertex_handle(id)).data());
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::ColorType OpenMeshTraits<OpenMeshType>::vertexColor(const MeshType& mesh, VertexId id) {
	return ColorType(mesh.color(mesh.vertex_handle(id)).data());
}

template <class OpenMeshType>
inline typename OpenMeshTraits<OpenMeshType>::NormalType OpenMeshTraits<OpenMeshType>::faceNormal(const MeshType& mesh, FaceId id) {
	return NormalType(mesh.normal(mesh.face_handle(id)).data());
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::PositionType> OpenMeshTraits<OpenMeshType>::vertexPositions(const MeshType& mesh) {
	std::vector<PositionType> positions(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) positions[i++] = PositionType(mesh.point(it.handle()).data());
	return positions;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::NormalType> OpenMeshTraits<OpenMeshType>::vertexNormals(const MeshType& mesh) {
	std::vector<NormalType> normals(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) normals[i++] = NormalType(mesh.normal(it.handle()).data());
	return normals;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::ColorType> OpenMeshTraits<OpenMeshType>::vertexColors(const MeshType& mesh) {
	std::vector<ColorType> colors(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) colors[i++] = ColorType(mesh.color(it.handle()).data());
	return colors;
}

template <class OpenMeshType>
inline std::vector<typename OpenMeshTraits<OpenMeshType>::NormalType> OpenMeshTraits<OpenMeshType>::faceNormals(const MeshType& mesh) {
	std::vector<NormalType> normals(mesh.n_faces());
	unsigned int i=0;
	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) normals[i++] = NormalType(mesh.normal(it.handle()).data());
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
