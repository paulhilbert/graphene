/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef SELECTMODESVIS_H_
#define SELECTMODESVIS_H_

#include <include/config.h>

#include <Algorithm/Sets.h>

#include <FW/FWVisualizer.h>
#include <Library/Input/AreaSelect.h>
#include <Library/Input/PaintSelect.h>

namespace FW {

template <class Entities, class Entity>
class SelectModes : virtual public Visualizer {
	public:
		typedef std::shared_ptr<SelectModes> Ptr;
		typedef std::weak_ptr<SelectModes>   WPtr;
		typedef typename Entities::Ptr       EntitiesPtr;
		typedef std::vector<int>             IdxSet;
		typedef enum {
			METHOD_NONE   = 0,
			METHOD_AREA   = 1 << 0,
			METHOD_PAINT  = 1 << 1
		} Methods;

	public:
		SelectModes(std::string id, Methods exclude = METHOD_NONE);
		virtual ~SelectModes();

		void init(EntitiesPtr entities);
		void render(ShaderProgram& program);
		void addProperties();
		void registerEvents();
		void addModes();

	protected:
		void resetSelection();
		virtual bool resetSelectionRender() = 0;
		virtual bool isInsideSelection(const Entity& entity, Input::SelectionMethod::Ptr method, Methods activeMethod) = 0;
		virtual void updateSelectionRender(const IdxSet& selection) = 0;

	protected:
		Methods                  m_exclude;
		Methods                  m_activeMethod;
		IdxSet                   m_selection;
		Input::AreaSelect::Ptr   m_areaSelect;
		Input::PaintSelect::Ptr  m_paintSelect;
};

#include "SelectModes.inl"

} // FW

#endif /* SELECTMODESVIS_H_ */
