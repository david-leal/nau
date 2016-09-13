#include "dialogs/dlgLights.h"

#include "dialogs/propertyManager.h"

#include "nau.h"
#include "nau/attribute.h"
#include "nau/event/eventFactory.h" 


BEGIN_EVENT_TABLE(DlgLights, wxDialog)

	EVT_COMBOBOX(DLG_COMBO, DlgLights::OnListSelect)
	EVT_PG_CHANGED( DLG_PROPS, DlgLights::OnPropsChange )
	EVT_BUTTON(DLG_BUTTON_ADD, DlgLights::OnAdd)
	
END_EVENT_TABLE()

wxWindow *DlgLights::Parent = NULL;
DlgLights *DlgLights::Inst = NULL;


void DlgLights::SetParent(wxWindow *p) {

	Parent = p;
}

DlgLights* DlgLights::Instance () {

	if (Inst == NULL)
		Inst = new DlgLights();

	return Inst;
}


DlgLights::DlgLights()
	: wxDialog(DlgLights::Parent, -1, wxT("Nau - Lights"),wxDefaultPosition,wxDefaultSize,wxRESIZE_BORDER|wxDEFAULT_DIALOG_STYLE)
                
{
	wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);

	setupPanel(sizer,Parent);

	SetAutoLayout(TRUE);
	sizer->SetSizeHints(this);
	sizer->Fit(this);
    SetSizer(sizer);

	this->SetTitle(wxT("Nau - Lights"));
	this->SetSize(350,200);
}


void
DlgLights::notifyUpdate(Notification aNot, std::string lightName, std::string value) {

	// sends events on behalf of the light
	std::shared_ptr<nau::event_::IEventData> e= nau::event_::EventFactory::Create("String");
	if (aNot == NEW_LIGHT) {
		e->setData(&lightName);
		EVENTMANAGER->notifyEvent("NEW_LIGHT", lightName,"", e);
	}
	else  {
		e->setData(&value);
		EVENTMANAGER->notifyEvent("LIGHT_CHANGED", lightName ,"", e);
	}
}


void 
DlgLights::updateDlg() {

	setupGrid();
	if (RENDERMANAGER->getNumLights()) {
		updateList();
		m_List->SetSelection(0);
		update();
		m_parent->Refresh();	
	}
	else {
		m_PG->Disable();
		m_List->Disable();
	}


}

void 
DlgLights::setupPanel(wxSizer *siz, wxWindow *parent) {

	int count = RENDERMANAGER->getNumLights();

	/* TOP: COMBO with camera names */
	wxBoxSizer *sizH1 = new wxBoxSizer(wxHORIZONTAL);

	wxStaticText *stg1 =  new wxStaticText(this,-1,wxT("Light: "));
	m_List = new wxComboBox(this,DLG_COMBO,wxT(""),wxDefaultPosition,wxDefaultSize,0,NULL,wxCB_READONLY );

	if (count) {
		updateList();
		m_List->SetSelection(0);
	}
	sizH1->Add(stg1, 0, wxGROW|wxHORIZONTAL,5);
	sizH1->Add(m_List, 1, wxGROW|wxHORIZONTAL,5);
	siz->Add(sizH1, 0, wxGROW|wxALL, 15);

	/* MIDDLE: Property grid */


	m_PG = new wxPropertyGridManager(this, DLG_PROPS,
				wxDefaultPosition, wxDefaultSize,
				// These and other similar styles are automatically
				// passed to the embedded wxPropertyGrid.
				wxPG_BOLD_MODIFIED|wxPG_SPLITTER_AUTO_CENTER|
				// Plus defaults.
				wxPGMAN_DEFAULT_STYLE
           );

	m_PG->AddPage(wxT("Lights"));

	m_PG->SetSplitterLeft(true);
	siz->Add(m_PG,1, wxEXPAND|wxALL,5);

	/* BOTTOM: Add Light Button */

	wxBoxSizer *sizH3 = new wxBoxSizer(wxHORIZONTAL);

	m_BAdd = new wxButton(this,DLG_BUTTON_ADD,wxT("Add Light"));
	sizH3-> Add(m_BAdd,0,wxALL |wxGROW |wxHORIZONTAL|wxALIGN_CENTER_HORIZONTAL,5);

	siz->Add(sizH3,0,wxALL |wxGROW |wxHORIZONTAL,15);

	if (!count) {
		m_PG->Disable();
		m_List->Disable();
	}

}


void
DlgLights::setupGrid() {

	m_PG->Clear();
	m_PG->AddPage(wxT("properties"));

	std::vector<std::string> order = { "ENABLED", "POSITION", "DIRECTION", "AMBIENT", "COLOR" };

	PropertyManager::createOrderedGrid(m_PG, Light::Attribs, order);
	m_PG->SetSplitterLeft(true, true);

}


void 
DlgLights::updateList() {

	std::vector<std::string> names;
	RENDERMANAGER->getLightNames(&names);
	int num = names.size();

	m_List->Clear();

	for(int i = 0; i < num; i++)  {
		wxString s;
		s << i;
		m_List->Append(wxString(names[i].c_str()));
	}
	m_active = names[0];
}



void 
DlgLights::update() {


	nau::scene::Light *light;		
	light = RENDERMANAGER->getLight(m_active).get();
//	unsigned int type = light->getPrope(Light::TYPE);

	m_PG->ClearSelection();

	PropertyManager::updateGrid(m_PG, Light::Attribs, (AttributeValues *)light);

}



void DlgLights::OnPropsChange( wxPropertyGridEvent& e) {

	nau::scene::Light *light = RENDERMANAGER->getLight(m_active).get();
	const wxString& name = e.GetPropertyName();
	unsigned int dotLocation = name.find_first_of(wxT("."),0);
	std::string topProp = std::string(name.substr(0,dotLocation).mb_str());
	std::string prop = std::string(name.substr(dotLocation+1,name.size()-dotLocation-1).mb_str());
	wxVariant variant;
	wxColour col;

	PropertyManager::updateProp(m_PG, topProp, Light::Attribs, (AttributeValues *)light);
	notifyUpdate(PROPS_CHANGED,m_active,topProp);
}


void DlgLights::updateInfo(std::string name) {

	if (name == m_active) {
		update();
	}
}


void DlgLights::OnAdd(wxCommandEvent& event) {

	int result;
	bool nameUnique,exit = false;
	std::string name;

	do {
		wxTextEntryDialog dialog(this,
								 _T("(the name must be unique)\n")
								 _T("Please Input a Light Name"),
								 _T("Light Name\n"),
								 _T(""),
								 wxOK | wxCANCEL);

		result = dialog.ShowModal();
		name = std::string(dialog.GetValue().mb_str());
		nameUnique =  !RENDERMANAGER->hasLight(name); 

		if (!nameUnique && (result == wxID_OK)){
			wxMessageBox(_T("Light name must be unique") , _T("Light Name Error"), wxOK | wxICON_INFORMATION, this);
		}
		if (name == "" && (result == wxID_OK)){
			wxMessageBox(_T("Light name must not be void"), _T("Light Name Error"), wxOK | wxICON_INFORMATION, this);
		}

		if (result == wxID_CANCEL) {
			exit = true;
		}
		else if (nameUnique && name != "") {
			exit = true;
		}

	} while (!exit);

	if (result == wxID_OK) {
		RENDERMANAGER->getLight(name);
		m_List->Enable();
		m_PG->Enable();
		updateList();
		m_List->Select(m_List->FindString(wxString(name.c_str())));
		m_active = name;
		update();
		notifyUpdate(NEW_LIGHT,name,"");
	}
}

void DlgLights::OnListSelect(wxCommandEvent& event){

	int sel;

	sel = event.GetSelection();
	m_active = std::string(m_List->GetString(sel).mb_str());
	update();
//	parent->Refresh();
}
