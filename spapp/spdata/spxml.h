//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_XML_H__
#define __SP_XML_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spfile.h"

namespace sp {

	//--------------------------------------------------------------------------------
	// xml
	//--------------------------------------------------------------------------------

	using namespace std;

	class XML {
	
	public:

		struct Unit {
			Unit *parent;
			Mem1<Unit*> child;

			string tag;
			string val;

			Unit() {
				parent = NULL;
			}
			Unit(Unit *parent, const char *tag, const char *val = NULL) {
				SP_ASSERT(parent != NULL || tag != NULL);

				this->parent = parent;
				this->tag = tag;
				this->val = (val != NULL) ? val : "";
			}
			Unit(const Unit &unit) {
				copy(unit);
			}
			void copy(const Unit &unit) {
				tag = unit.tag;
				val = unit.val;
				parent = unit.parent;
				child = unit.child;
			}
		};

		Unit *m_crnt;
		MemP<Unit> m_units;

	public:
		XML() {
			reset();
		}

		void reset() {
			m_units.clear();
			m_crnt = m_units.malloc();
		}

		void begin(const char *tag) {

			SP_ASSERT(tag != NULL);

			Unit *unit = m_units.malloc();
			unit->parent = m_crnt;
			unit->tag = tag;

			m_crnt->child.push(unit);
			m_crnt = unit;
		}

		void add(const char *tag, const char *val) {
			SP_ASSERT(tag != NULL && val != NULL);

			Unit *unit = m_units.malloc();
			unit->parent = m_crnt;
			unit->tag = tag;
			unit->val = val;

			m_crnt->child.push(unit);
		}

		void end() {
			SP_ASSERT(m_crnt != NULL);
			m_crnt = m_crnt->parent;
		}

	public:

		void save(const char *path) {
			File file(path, "w");

			Unit *root = &m_units[0];
			for (int i = 0; i < root->child.size(); i++) {
				save(file, root->child[i], 0);
			}
		}

		void load(const char *path) {
			File file(path, "r");

			Unit *root = &m_units[0];
			for (int i = 0; i < root->child.size(); i++) {
				save(file, root->child[i], 0);
			}
		}

	private:

		void save(File &file, const Unit *unit, const int level) {
			char tab[SP_STRMAX] = { 0 };
			for (int i = 0; i < level; i++) {
				strcat(tab, "\t");
			}

			const char *tag = unit->tag.c_str();
			const char *val = unit->val.c_str();
				
			file.printf("%s<%s>", tab, tag);
			if (unit->child.size() == 0) {
				file.printf("%s", val);
			}
			else {
				file.printf("\n");
				for (int i = 0; i < unit->child.size(); i++) {
					save(file, unit->child[i], level + 1);
				}
				file.printf("%s", tab);
			}
			file.printf("</%s>\n", tag);
		}
	};


}

#endif