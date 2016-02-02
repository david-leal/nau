#include "dlgMaterials.h"

#include "propertyManager.h"
#include "dlgTextureLib.h"

#include <nau.h>
#include <nau/event/eventFactory.h>
#include <nau/loader/projectLoader.h>


BEGIN_EVENT_TABLE(DlgMaterials, wxDialog)


	EVT_CLOSE(DlgMaterials::OnClose)

	EVT_COMBOBOX(DLG_MI_COMBO_MATERIAL, DlgMaterials::OnSelectMaterial)
	EVT_COMBOBOX(DLG_MI_COMBO_LIBMATERIAL, DlgMaterials::OnSelectLibMaterial)

	EVT_PG_CHANGED( DLG_MI_PGMAT, DlgMaterials::OnProcessColorChange )
	EVT_PG_CHANGED( DLG_MI_PGTEXTPROPS, DlgMaterials::OnProcessTexturePropsChange )
	EVT_GRID_CMD_CELL_LEFT_CLICK(DLG_MI_TEXTURE_GRID,DlgMaterials::OnprocessClickGrid)

	EVT_COMBOBOX(DLG_SHADER_COMBO, DlgMaterials::OnShaderListSelect)

	EVT_CHECKBOX(DLG_SHADER_USE,DlgMaterials::OnProcessUseShader)
	EVT_PG_CHANGED( DLG_SHADER_UNIFORMS, DlgMaterials::OnProcessShaderUpdateUniforms )
	
	EVT_PG_CHANGED( DlgMatStatePanels::PG, DlgMaterials::OnProcessPanelChange )

	EVT_PG_CHANGED( DlgMatImageTexturePanels::PG, DlgMaterials::OnProcessITexPanelChange )
	EVT_COMBOBOX( DlgMatImageTexturePanels::DLG_ITEM_COMBO, DlgMaterials::OnProcessITexPanelSelect )

	EVT_PG_CHANGED( DlgMatBufferPanels::PG, DlgMaterials::OnProcessBufferPanelChange )
	EVT_COMBOBOX( DlgMatBufferPanels::DLG_ITEM_COMBO, DlgMaterials::OnProcessBufferPanelSelect )

    EVT_MENU(LIBMAT_NEW, DlgMaterials::toolbarLibMatNew)
    EVT_MENU(LIBMAT_SAVE, DlgMaterials::toolbarLibMatSave)
    EVT_MENU(LIBMAT_SAVEALL, DlgMaterials::toolbarLibMatSaveAll)
    EVT_MENU(LIBMAT_OPEN, DlgMaterials::toolbarLibMatOpen)

	EVT_MENU(MAT_NEW, DlgMaterials::toolbarMatNew)
	EVT_MENU(MAT_CLONE, DlgMaterials::toolbarMatClone)
	EVT_MENU(MAT_REMOVE, DlgMaterials::toolbarMatRemove)
	EVT_MENU(MAT_COPY, DlgMaterials::toolbarMatCopy)
	EVT_MENU(MAT_PASTE, DlgMaterials::toolbarMatPaste)

END_EVENT_TABLE()



// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------


wxWindow *DlgMaterials::parent = NULL;
DlgMaterials *DlgMaterials::inst = NULL;

void DlgMaterials::SetParent(wxWindow *p) {

	parent = p;
}


DlgMaterials* DlgMaterials::Instance () {

	if (inst == NULL)
		inst = new DlgMaterials();

	return inst;

}


DlgMaterials::~DlgMaterials() {

	delete m_EmptyBitmap;
}


DlgMaterials::DlgMaterials()
	: wxDialog(DlgMaterials::parent, -1, wxT("Nau - Materials"), wxDefaultPosition, wxDefaultSize, wxRESIZE_BORDER | wxDEFAULT_DIALOG_STYLE),
	samplerUnits(8), m_Name("Materials Dialog")
{

	m_parent = DlgMaterials::parent;

	m_copyMat = NULL;
	m_EmptyBitmap = new wxBitmap(96, 96);

	/* ----------------------------------------------------------------
								 Toolbar
	-----------------------------------------------------------------*/

	// load the image wanted on the toolbar
	wxBitmap openImage(wxT("bitmaps/open.bmp"), wxBITMAP_TYPE_BMP);
	wxBitmap newImage(wxT("bitmaps/new.bmp"), wxBITMAP_TYPE_BMP);
	wxBitmap saveImage(wxT("bitmaps/save.bmp"), wxBITMAP_TYPE_BMP);


	wxBitmap newMImage(wxT("bitmaps/project.bmp"), wxBITMAP_TYPE_BMP);
	wxBitmap cloneImage(wxT("bitmaps/project.bmp"), wxBITMAP_TYPE_BMP);
	wxBitmap cutImage(wxT("bitmaps/project.bmp"), wxBITMAP_TYPE_BMP);
	wxBitmap copyImage(wxT("bitmaps/project.bmp"), wxBITMAP_TYPE_BMP);
	wxBitmap pasteImage(wxT("bitmaps/project.bmp"), wxBITMAP_TYPE_BMP);

	// create the toolbar and add our 1 tool to it
	m_toolbar = new wxToolBar(this, TOOLBAR_ID);
	long tstyle = m_toolbar->GetWindowStyle();
	tstyle |= wxTB_TEXT;
	m_toolbar->SetWindowStyle(tstyle);
	m_toolbar->AddTool(LIBMAT_NEW, _("New"), newImage, _("New Material Lib"));
	m_toolbar->AddTool(LIBMAT_OPEN, _("Open"), openImage, _("Open Material Lib"));
	m_toolbar->AddTool(LIBMAT_SAVE, _("Save"), saveImage, _("Save Material Lib"));
	m_toolbar->AddTool(LIBMAT_SAVEALL, _("Save All"), newImage, _("Save All Libs"));
	m_toolbar->AddSeparator();
	m_toolbar->AddTool(MAT_NEW, _("New"), newImage, _("New Material"));
	m_toolbar->AddTool(MAT_CLONE, _("Clone"), cloneImage, _("Clone Material"));
	m_toolbar->AddTool(MAT_REMOVE, _("Remove"), cutImage, _("Remove Material"));
	m_toolbar->AddTool(MAT_COPY, _("Copy"), copyImage, _("Copy Material"));
	m_toolbar->AddTool(MAT_PASTE, _("Paste"), pasteImage, _("Paste Material"));

	m_toolbar->EnableTool(LIBMAT_SAVE, FALSE);
	m_toolbar->EnableTool(LIBMAT_SAVEALL, FALSE);
	m_toolbar->EnableTool(MAT_NEW, FALSE);
	m_toolbar->EnableTool(MAT_CLONE, FALSE);
	m_toolbar->EnableTool(MAT_REMOVE, FALSE);
	m_toolbar->EnableTool(MAT_PASTE, FALSE);

	m_toolbar->Realize();


	wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
	sizer->Add(m_toolbar, 0, wxGROW | wxALL, 1);

	/* ----------------------------------------------------------------
								 Materials
	-----------------------------------------------------------------*/


	/* --- Material List ----------------------------------------------*/

	libList = new wxComboBox(this, DLG_MI_COMBO_LIBMATERIAL, wxT(""), wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY);

	std::vector<std::string> libs;
	MATERIALLIBMANAGER->getNonEmptyLibNames(&libs);

	for (auto& lib : libs) {
		if (MATERIALLIBMANAGER->getLib(lib)->getMaterialCount())
			libList->Append(wxString(lib.c_str()));
	}
	libList->SetSelection(0);

	materialList = new wxComboBox(this, DLG_MI_COMBO_MATERIAL, wxT(""), wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY);

	std::vector<std::string> mats;
	MATERIALLIBMANAGER->getMaterialNames(libs[0], &mats);

	for (auto& mat : mats)
		materialList->Append(wxString(mat.c_str()));

	materialList->SetSelection(0);

	wxBoxSizer *sizer1 = new wxBoxSizer(wxHORIZONTAL);
	wxStaticText *stg1 =  new wxStaticText(this,-1,wxT("Library: "));

	sizer1->Add(stg1,0,wxGROW |wxALL, 5);
	sizer1->Add(libList,0,wxGROW |wxALL,5);
	wxStaticText *stg2 =  new wxStaticText(this,-1,wxT("Materials: "));
	sizer1->Add(stg2,0,wxGROW |wxALL, 5);
	sizer1->Add(materialList,0,wxGROW |wxALL,5);
	sizer1->Add(5,5,0, wxGROW |wxALL,5);

	sizer->Add(sizer1,0,wxGROW | wxALL,5);
	/* -----------------------------------------------------------------
								NoteBook
	------------------------------------------------------------------*/

	wxNotebook *notebook = new wxNotebook(this,-1);
	//wxNotebookSizer *nbSizer = new wxNotebookSizer(notebook);

	/* Colors */
	wxPanel *pColors = new wxPanel(notebook,-1);
	notebook->AddPage(pColors,wxT("Colors"));

	wxSizer *colors = new wxBoxSizer(wxVERTICAL);

	setupColorPanel(colors,pColors);

	pColors->SetAutoLayout(TRUE);
	pColors->SetSizer(colors);


	/* Shaders */
	wxPanel *pShaders = new wxPanel(notebook,-1);
	notebook->AddPage(pShaders,wxT("Shaders"));

	wxSizer *shaders = new wxBoxSizer(wxVERTICAL);

	setupShaderPanel(shaders,pShaders);
//	setupShaders();

	pShaders->SetAutoLayout(TRUE);
	pShaders->SetSizer(shaders);



	/* Textures */
	wxPanel *pTextures = new wxPanel(notebook,-1);
	notebook->AddPage(pTextures,wxT("Textures"));

	wxSizer *textures = new wxBoxSizer(wxVERTICAL);

	setupTexturesPanel(textures,pTextures);

	pTextures->SetAutoLayout(TRUE);
	pTextures->SetSizer(textures);


	/* State */
	wxPanel *pOGL = new wxPanel(notebook,-1);
	notebook->AddPage(pOGL,wxT("State"));

	wxSizer *OGL = new wxBoxSizer(wxVERTICAL);

	std::shared_ptr<Material> &mm = getModelMaterial();
	if (mm) {
		panels.setState(mm->getState());
	}
	panels.setPanel(OGL,pOGL);

	pOGL->SetAutoLayout(TRUE);
	pOGL->SetSizer(OGL);

	/* Buffers */
	wxPanel *pBuffers = new wxPanel(notebook,-1);
	notebook->AddPage(pBuffers,wxT("Buffers"));

	wxSizer *Buffers = new wxBoxSizer(wxVERTICAL);

	m_BufferPanel.setPanel(Buffers,pBuffers);
	m_BufferPanel.setMaterial(getModelMaterial());

	pBuffers->SetAutoLayout(TRUE);
	pBuffers->SetSizer(Buffers);

	/* Image Textures */
	wxPanel *pITexs = new wxPanel(notebook,-1);
	
	notebook->AddPage(pITexs,wxT("Image Textures"));
	wxSizer *ITexs = new wxBoxSizer(wxVERTICAL);

	m_ITexPanel.setPanel(ITexs,pITexs);
	m_ITexPanel.setMaterial(getModelMaterial());

	pITexs->SetAutoLayout(TRUE);
	pITexs->SetSizer(ITexs);

	/* -----------------------------------------------------------------
								NoteBook End
	------------------------------------------------------------------*/

	sizer->Add(notebook,1,wxEXPAND | wxALL ,5);
	
	/* ----------------------------------------------------------------
	                             Materials End
	-----------------------------------------------------------------*/


	sizer->Add(5,5,0, wxGROW | wxALL,5);

    SetAutoLayout(TRUE);
    SetSizer(sizer);

    sizer->SetSizeHints(this);
    sizer->Fit(this);


}


void DlgMaterials::OnClose(wxCloseEvent& event) {
	
	this->Hide();
}


void DlgMaterials::updateDlg() {

	EVENTMANAGER->addListener("NEW_LIGHT",this);
	EVENTMANAGER->addListener("LIGHT_CHANGED",this);
	EVENTMANAGER->addListener("NEW_SHADER",this);
	EVENTMANAGER->addListener("SHADER_CHANGED",this);
	EVENTMANAGER->addListener("NEW_CAMERA",this);
	EVENTMANAGER->addListener("CAMERA_CHANGED",this);
	EVENTMANAGER->addListener("NEW_TEXTURE",this);
	resetTexturePropGrid();
	panels.resetPropGrid();
	resetColorPanel();
	updateMaterialList();
}


std::shared_ptr<Material> &
DlgMaterials::getModelMaterial() {

	std::string mat, lib;
	
	lib = std::string(libList->GetValue().mb_str());
	mat = std::string(materialList->GetValue().mb_str());

	std::shared_ptr<Material> &mm = MATERIALLIBMANAGER->getMaterial(lib,mat);

	return mm;
}


void DlgMaterials::OnSelectMaterial(wxCommandEvent& event) {

	std::shared_ptr<Material> &mm = getModelMaterial();

	updateColors(mm) ;
	updateTextures(mm,0);
	updateShader(mm);
	panels.setState(mm->getState());
	panels.updatePanel();
	m_BufferPanel.setMaterial(mm);
	m_ITexPanel.setMaterial(mm);

//	Refresh();


}


void DlgMaterials::OnSelectLibMaterial(wxCommandEvent& event) {

	int sel;
	sel = event.GetSelection();

	std::vector<std::string> libs;
	MATERIALLIBMANAGER->getNonEmptyLibNames(&libs);
	std::vector<std::string> mats;
	MATERIALLIBMANAGER->getMaterialNames(libs.at(sel), &mats);

	materialList->Clear();
	for (auto& mat : mats)
		materialList->Append(wxString(mat.c_str()));

	materialList->SetSelection(0);

	std::shared_ptr<Material> &mm = getModelMaterial();
	if (mm) {
		updateColors(mm);
		updateTextures(mm, 0);
		updateShader(mm);
		panels.setState(mm->getState());
		panels.updatePanel();
		m_BufferPanel.setMaterial(mm);
		m_ITexPanel.setMaterial(mm);
	}
	else {
	}

	if (libs.at(sel).substr(0,1) == " ") {
		m_toolbar->EnableTool(LIBMAT_SAVE, FALSE);
		m_toolbar->EnableTool(MAT_NEW,FALSE);
		m_toolbar->EnableTool(MAT_CLONE,FALSE);
		m_toolbar->EnableTool(MAT_REMOVE,FALSE);
		m_toolbar->EnableTool(MAT_PASTE,FALSE);
	}
	else {
		m_toolbar->EnableTool(LIBMAT_SAVEALL, TRUE);
		m_toolbar->EnableTool(LIBMAT_SAVE, TRUE);
		m_toolbar->EnableTool(MAT_NEW,TRUE);
		m_toolbar->EnableTool(MAT_CLONE,TRUE);
		if (materialList->GetCount() > 1)
			m_toolbar->EnableTool(MAT_REMOVE,TRUE);
		else
			m_toolbar->EnableTool(MAT_REMOVE,FALSE);
		if (m_copyMat)
			m_toolbar->EnableTool(MAT_PASTE,TRUE);
		
	}
}



void DlgMaterials::updateMaterialList() {

	std::vector<std::string> libs;
	MATERIALLIBMANAGER->getNonEmptyLibNames(&libs);

	wxString sel = libList->GetStringSelection();
	libList->Clear();
	for (auto& lib:libs)
		libList->Append(wxString(lib.c_str()));

	if (! libList->SetStringSelection(sel))
		libList->SetSelection(0);

	std::vector<std::string> mats;
	MATERIALLIBMANAGER->getMaterialNames(libs.at(libList->GetCurrentSelection()), &mats);

	sel = materialList->GetStringSelection();
	materialList->Clear();
	for (auto &mat:mats)
		materialList->Append(wxString(mat.c_str()));
	if (! materialList->SetStringSelection(sel))
		materialList->SetSelection(0);

	std::shared_ptr<Material> &mm = getModelMaterial();
	if (mm) {
		updateColors(mm);
		updateTextures(mm, 0);
		updateShader(mm);
		panels.setState(mm->getState());
		panels.updatePanel();
		m_BufferPanel.setMaterial(mm);
		m_ITexPanel.setMaterial(mm);
	}
}


void DlgMaterials::OnProcessPanelChange( wxPropertyGridEvent& e) {

	panels.OnProcessPanelChange(e);
}

/* ----------------- BUFFERS -----------------------------------*/

void DlgMaterials::OnProcessBufferPanelChange( wxPropertyGridEvent& e) {

	m_BufferPanel.onProcessPanelChange(e);
}


void DlgMaterials::OnProcessBufferPanelSelect( wxCommandEvent& e) {

	m_BufferPanel.onItemListSelect(e);
}


/* ----------------- IMAGE TEXTURES ----------------------------*/


void DlgMaterials::OnProcessITexPanelChange( wxPropertyGridEvent& e) {

	m_ITexPanel.onProcessPanelChange(e);
}


void DlgMaterials::OnProcessITexPanelSelect( wxCommandEvent& e) {

	m_ITexPanel.onItemListSelect(e);
}

/* ----------------------------------------------------------------

				TOOLBAR STUFF

-----------------------------------------------------------------*/

void DlgMaterials::toolbarLibMatNew(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarLibMatOpen(wxCommandEvent& WXUNUSED(event) ) {


	std::string fullName;

	wxFileDialog dialog
                 (
                    this,
                    _T("Open Material Lib"),
                    _T(""),
                    _T(""),
                    _T("Material Library (*.mlib)|*.mlib"),
					wxFD_OPEN | wxFD_FILE_MUST_EXIST 
                 );

	if (dialog.ShowModal() != wxID_OK)
		return;

	fullName = std::string(dialog.GetPath().mb_str());
	
	nau::loader::ProjectLoader::loadMatLib(fullName);

	updateDlg();
	m_toolbar->EnableTool(LIBMAT_SAVEALL, TRUE);
	EVENTMANAGER->notifyEvent("NEW_MATERIAL", "","",NULL);
}


void DlgMaterials::toolbarLibMatSave(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarLibMatSaveAll(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarMatNew(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarMatPaste(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarMatClone(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarMatRemove(wxCommandEvent& WXUNUSED(event) ) {

}


void DlgMaterials::toolbarMatCopy(wxCommandEvent& WXUNUSED(event) ) {

}


/* ----------------------------------------------------------------

				EVENTS FROM OTHER DIALOGS AND NAU

-----------------------------------------------------------------*/

void 
DlgMaterials::eventReceived(const std::string &sender, const std::string &eventType, 
	const std::shared_ptr<nau::event_::IEventData> &evt) {

	std::string *str;
	std::string aux;
	std::shared_ptr<Material> &m = getModelMaterial();

	if (eventType == "NEW_LIGHT") {
		str = (std::string *)evt->getData();	
		m_pgLightList.Add(wxString(str->c_str()));
	}
	else if (eventType == "NEW_SHADER") {
		str = (std::string *)evt->getData();
		shaderList->AppendString(wxString(str->c_str()));
	}
	else if (eventType == "NEW_CAMERA") {
		str = (std::string *)evt->getData();
		m_pgCamList.Add(wxString(str->c_str()));
	}
	else if (eventType == "CAMERA_CHANGED") {

	}
	else if (eventType == "LIGHT_CHANGED") {

	}
	else if (eventType == "SHADER_CHANGED") {
		updateShader(getModelMaterial());
	}
	else if (eventType == "NEW_TEXTURE") {
		updateTextureList();
		updateTextures(m,0);
	}
	else if (eventType == "TEXTURE_ICON_UPDATE") {
		for(int i = 0; i < 2; i++) { 
			for(int j = 0 ; j < 4 ; j++){
				ITexture *texture = m->getTexture(i*4+j);
				imagesGrid[i * 4 + j]->setBitmap(DlgTextureLib::Instance()->m_Bitmaps[texture->getPropi(ITexture::ID)]);
			}
		}
	}
}


void 
DlgMaterials::setPropf4Aux(std::string &propNamex, vec4 &values) 
{
	wxString aux;
	wxString propName = wxString(propNamex.c_str());
	aux = propName + wxT(".fvalues.x");
	const wxChar *auxc = aux.c_str();

	pgShaderUniforms->SetPropertyValue(auxc,values.x);
	aux = propName + wxT(".fvalues.y");
	pgShaderUniforms->SetPropertyValue(auxc,values.y);
	aux = propName + wxT(".fvalues.z");
	pgShaderUniforms->SetPropertyValue(auxc,values.z);
	aux = propName + wxT(".fvalues.w");
	pgShaderUniforms->SetPropertyValue(auxc,values.w);
}


void
DlgMaterials::setPropm4Aux(std::string &propNamex, mat4 &m) 
{
	wxString aux;
	wxString propName = wxString(propNamex.c_str());
	aux = propName + wxT(".fvalues.Row 1.m11");
	const wxChar *auxc = aux.c_str();

	pgShaderUniforms->SetPropertyValue(auxc,m.at(1,1));
	aux = propName + wxT(".fvalues.Row 1.m12");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(1,2));
	aux = propName + wxT(".fvalues.Row 1.m13");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(1,3));
	aux = propName + wxT(".fvalues.Row 1.m14");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(1,4));

	aux = propName + wxT(".fvalues.Row 2.m21");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(2,1));
	aux = propName + wxT(".fvalues.Row 2.m22");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(2,2));
	aux = propName + wxT(".fvalues.Row 2.m23");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(2,3));
	aux = propName + wxT(".fvalues.Row 2.m24");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(2,4));

	aux = propName + wxT(".fvalues.Row 3.m31");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(3,1));
	aux = propName + wxT(".fvalues.Row 3.m32");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(3,2));
	aux = propName + wxT(".fvalues.Row 3.m33");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(3,3));
	aux = propName + wxT(".fvalues.Row 3.m34");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(3,4));

	aux = propName + wxT(".fvalues.Row 4.m41");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(4,1));
	aux = propName + wxT(".fvalues.Row 4.m42");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(4,2));
	aux = propName + wxT(".fvalues.Row 4.m43");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(4,3));
	aux = propName + wxT(".fvalues.Row 4.m44");
	pgShaderUniforms->SetPropertyValue(auxc,m.at(4,4));
}


/* ----------------------------------------------------------------------

	Textures Stuff

-----------------------------------------------------------------------*/


void DlgMaterials::setupTexturesPanel(wxSizer *siz, wxWindow *parent) {

	std::shared_ptr<Material> &mm = getModelMaterial();

	gridTextures = new wxGrid(parent,DLG_MI_TEXTURE_GRID,wxDefaultPosition,
                       wxSize( 420, 210 ));

	gridTextures->CreateGrid(2,4);
	gridTextures->SetColLabelSize(0);
	gridTextures->SetRowLabelSize(0);
	gridTextures->SetColSize(0,100);
	gridTextures->SetColSize(1,100);
	gridTextures->SetColSize(2,100);
	gridTextures->SetColSize(3,100);
	gridTextures->SetRowMinimalAcceptableHeight(100);
	gridTextures->SetColMinimalAcceptableWidth(100);

	ITexture *texture;
	for(int i = 0; i < 2; i++) { 
		gridTextures->SetRowSize(i,100);
		for(int j = 0 ; j < 4 ; j++){
			if (mm)
				texture = mm->getTexture(i * 4 + j);
			else
				texture = NULL;
	
			gridTextures->SetReadOnly(i,j,true);
			if (texture != NULL)
				imagesGrid[i * 4 + j] = new ImageGridCellRenderer(DlgTextureLib::Instance()->m_Bitmaps[texture->getPropi(ITexture::ID)]);
			else
				imagesGrid[i*4+j] = new ImageGridCellRenderer(m_EmptyBitmap);
			gridTextures->SetCellRenderer(i, j, imagesGrid[i*4+j]);
		}
	}

	wxStaticBox *texProp = new wxStaticBox(parent,-1,wxT("Texture Properties"));
	wxSizer *sizerTP = new wxStaticBoxSizer(texProp,wxVERTICAL);

	pgTextureProps = new wxPropertyGridManager(parent, DLG_MI_PGTEXTPROPS,
				wxDefaultPosition, wxDefaultSize,
				// These and other similar styles are automatically
				// passed to the embedded wxPropertyGrid.
				wxPG_BOLD_MODIFIED|wxPG_SPLITTER_AUTO_CENTER|
				// Plus defaults.
				wxPGMAN_DEFAULT_STYLE
           );


	pgTextureProps->AddPage(wxT("Texture"));

			
	const wxChar* units[] = {wxT("0"),wxT("1"),wxT("2"),wxT("3"),wxT("4"),wxT("5"),wxT("6"),wxT("7"),NULL};
	const long unitsInd[] = {0,1,2,3,4,5,6,7};

	const wxChar* texType[] = { wxT("TEXTURE_1D"), wxT("TEXTURE_2D"), wxT("TEXTURE_3D"), wxT("TEXTURE_CUBE_MAP"),NULL};
	const long texTypeInd[] = { (long)GL_TEXTURE_1D,(long)GL_TEXTURE_2D,(long)GL_TEXTURE_3D, (long)GL_TEXTURE_CUBE_MAP};

	pgTextureProps->Append( new wxEnumProperty(wxT("Texture Unit"),wxPG_LABEL,units,unitsInd,0));

	wxPGProperty *pgid;
	textureLabels.Add(wxT("No Texture"),-1);
	m_pgPropTextureList = new wxEnumProperty(wxT("Name"),wxPG_LABEL,textureLabels);
	pgid = pgTextureProps->Append(m_pgPropTextureList);
	updateTextureList();

	pgTextureProps->Append( new wxEnumProperty(wxT("Texture Type"),wxPG_LABEL,texType,texTypeInd,(int)GL_TEXTURE_2D));
	pgTextureProps->DisableProperty(wxT("Texture Type"));

	wxString texDim;
	texDim.Printf(wxT("%d x %d x %d"),0,0,0);
	pgTextureProps->Append( new wxStringProperty(wxT("Dimensions(WxHxD)"),wxPG_LABEL,texDim));
 	pgTextureProps->DisableProperty(wxT("Dimensions(WxHxD)"));
	std::vector<std::string> order = {"WRAP_S", "WRAP_T", "WRAP_R", "MIN_FILTER", "MAG_FILTER", "COMPARE_MODE",
			"COMPARE_FUNC", "BORDER_COLOR"};
	PropertyManager::createOrderedGrid(pgTextureProps, ITextureSampler::Attribs,order);

	sizerTP->Add(pgTextureProps,1,wxEXPAND,15);

	siz->Add(gridTextures,0,wxALL|wxALIGN_CENTER,5);

	siz->Add(sizerTP,1,wxALL|wxGROW,5);
	if (mm)
		updateTextures(mm,0);
}


void 
DlgMaterials::resetTexturePropGrid() {

	pgTextureProps->Clear();
	pgTextureProps->AddPage(wxT("Texture"));


	const wxChar* units[] = { wxT("0"),wxT("1"),wxT("2"),wxT("3"),wxT("4"),wxT("5"),wxT("6"),wxT("7"),NULL };
	const long unitsInd[] = { 0,1,2,3,4,5,6,7 };

	const wxChar* texType[] = { wxT("TEXTURE_1D"), wxT("TEXTURE_2D"), wxT("TEXTURE_3D"), wxT("TEXTURE_CUBE_MAP"),NULL };
	const long texTypeInd[] = { (long)GL_TEXTURE_1D,(long)GL_TEXTURE_2D,(long)GL_TEXTURE_3D, (long)GL_TEXTURE_CUBE_MAP };

	pgTextureProps->Append(new wxEnumProperty(wxT("Texture Unit"), wxPG_LABEL, units, unitsInd, 0));

	wxPGProperty *pgid;
	textureLabels.Add(wxT("No Texture"), -1);
	m_pgPropTextureList = new wxEnumProperty(wxT("Name"), wxPG_LABEL, textureLabels);
	pgid = pgTextureProps->Append(m_pgPropTextureList);
	updateTextureList();

	pgTextureProps->Append(new wxEnumProperty(wxT("Texture Type"), wxPG_LABEL, texType, texTypeInd, (int)GL_TEXTURE_2D));
	pgTextureProps->DisableProperty(wxT("Texture Type"));

	wxString texDim;
	texDim.Printf(wxT("%d x %d x %d"), 0, 0, 0);
	pgTextureProps->Append(new wxStringProperty(wxT("Dimensions(WxHxD)"), wxPG_LABEL, texDim));
	pgTextureProps->DisableProperty(wxT("Dimensions(WxHxD)"));
	std::vector<std::string> order = { "WRAP_S", "WRAP_T", "WRAP_R", "MIN_FILTER", "MAG_FILTER", "COMPARE_MODE",
		"COMPARE_FUNC", "BORDER_COLOR" };
	PropertyManager::createOrderedGrid(pgTextureProps, ITextureSampler::Attribs, order);

}

void DlgMaterials::setTextureUnit(int index){

	std::shared_ptr<Material> &mm = getModelMaterial();
	IState *state = mm->getState();
	ITexture *texture = mm->getTexture(index);
	ITextureSampler *ts = mm->getTextureSampler(index);

	pgTextureProps->SetPropertyValue(wxT("Texture Unit"),(long)index);

	int row,col;
	row = index / 4;
	col = index % 4;
	gridTextures->SelectBlock(row,col,row,col,false);

	if (texture == NULL)
		pgTextureProps->SetPropertyValueString(wxT("Name"),wxT("No Texture"));
	else
		pgTextureProps->SetPropertyValueString(wxT("Name"), wxString(texture->getLabel().c_str()));


	if (texture == NULL) {
		pgTextureProps->SelectProperty(wxT("Texture Unit"));
	 	pgTextureProps->DisableProperty(wxT("WRAP_S"));
	 	pgTextureProps->DisableProperty(wxT("WRAP_T"));
	 	pgTextureProps->DisableProperty(wxT("WRAP_R"));
	 	pgTextureProps->DisableProperty(wxT("MAG_FILTER"));
	 	pgTextureProps->DisableProperty(wxT("MIN_FILTER"));
		pgTextureProps->DisableProperty(wxT("COMPARE_MODE"));
		pgTextureProps->DisableProperty(wxT("COMPARE_FUNC"));
		pgTextureProps->DisableProperty(wxT("BORDER_COLOR"));
		pgTextureProps->DisableProperty(wxT("ID"));
		pgTextureProps->DisableProperty(wxT("MIPMAP"));
		return;
	}
	
	wxString texDim;
	texDim.Printf(wxT("%d x %d x %d"),texture->getPropi(ITexture::WIDTH),
		texture->getPropi(ITexture::HEIGHT),
								texture->getPropi(ITexture::DEPTH));
	pgTextureProps->SetPropertyValue(wxT("Dimensions(WxHxD)"),texDim);

	pgTextureProps->SetPropertyValue(wxT("Texture Type"),wxString(ITexture::Attribs.getListStringOp(ITexture::DIMENSION, texture->getPrope(ITexture::DIMENSION)).c_str()));

	pgTextureProps->EnableProperty(wxT("MAG_FILTER"));
	pgTextureProps->EnableProperty(wxT("MIN_FILTER"));
	pgTextureProps->EnableProperty(wxT("COMPARE_MODE"));
	pgTextureProps->EnableProperty(wxT("COMPARE_FUNC"));
	pgTextureProps->EnableProperty(wxT("BORDER_COLOR"));	

	//pgTextureProps->SetPropertyValue(wxT("WRAP_S"),(long)ts->getPrope(ITextureSampler::WRAP_S));
	//pgTextureProps->SetPropertyValue(wxT("WRAP_T"),(long)ts->getPrope(ITextureSampler::WRAP_T));
	//pgTextureProps->SetPropertyValue(wxT("WRAP_R"),(long)ts->getPrope(ITextureSampler::WRAP_R));

	if (texture->getPrope(ITexture::DIMENSION) == GL_TEXTURE_1D) {

		pgTextureProps->EnableProperty(wxT("WRAP_S"));
	 	pgTextureProps->DisableProperty(wxT("WRAP_T"));
	 	pgTextureProps->DisableProperty(wxT("WRAP_R"));
	}
	else if (texture->getPrope(ITexture::DIMENSION) == GL_TEXTURE_2D || texture->getPrope(ITexture::DIMENSION) == GL_TEXTURE_CUBE_MAP)  {
		pgTextureProps->EnableProperty(wxT("WRAP_S"));
		pgTextureProps->EnableProperty(wxT("WRAP_T"));
	 	pgTextureProps->DisableProperty(wxT("WRAP_R"));
	}
	else {
		pgTextureProps->EnableProperty(wxT("WRAP_S"));
		pgTextureProps->EnableProperty(wxT("WRAP_T"));
		pgTextureProps->EnableProperty(wxT("WRAP_R"));
	}
	PropertyManager::updateGrid(pgTextureProps, ITextureSampler::Attribs, (AttributeValues *)ts);
	//pgTextureProps->SetPropertyValue(wxT("MAG_FILTER"),(long)ts->getPrope(ITextureSampler::MAG_FILTER));
	//pgTextureProps->SetPropertyValue(wxT("MIN_FILTER"),(long)ts->getPrope(ITextureSampler::MIN_FILTER));

	//pgTextureProps->SetPropertyValue(wxT("COMPARE_MODE"),(long)ts->getPrope(ITextureSampler::COMPARE_MODE));
	//pgTextureProps->SetPropertyValue(wxT("COMPARE_FUNC"),(long)ts->getPrope(ITextureSampler::COMPARE_FUNC));
}



void DlgMaterials::OnProcessTexturePropsChange( wxPropertyGridEvent& e) {

	std::shared_ptr<Material> &mm = getModelMaterial();
	IState *state = mm->getState();

	wxString& name = e.GetPropertyName();
	wxPGProperty *topProp = e.GetProperty()->GetParent();
	const wxString& topName = e.GetProperty()->GetParent()->GetName();
	if (topName != "<Root>")
		name = topName;
	const int index = pgTextureProps->GetPropertyValueAsLong(wxT("Texture Unit"));
	//const int value = pgTextureProps->GetPropertyValueAsLong(name);

	ITexture *texture = mm->getTexture(index);
	ITextureSampler *ts = mm->getTextureSampler(index);

	if (name == wxT("Texture Unit")) {
		
		setTextureUnit(index);
	}
	else if (name == wxT("Name")) {

		int value = pgTextureProps->GetPropertyValueAsInt(wxT("Name"));
		if (value == -1) {
			if (samplerUnits[index] == 0) {
				mm->unsetTexture(index);
				imagesGrid[index]->setBitmap(new wxBitmap(96, 96));
			}
			else {
				wxMessageBox(wxT("Texture unit is assigned to a shader sampler and can't be unset"), wxT("Texture setting error"));
				std::string s = mm->getTexture(index)->getLabel();
				pgTextureProps->SetPropertyValueString(wxT("Name"), wxString(s.c_str()));
			}
		}
		else {
			ITexture *t = RESOURCEMANAGER->getTexture(value);
			mm->attachTexture(index, t);
			imagesGrid[index]->setBitmap(DlgTextureLib::Instance()->m_Bitmaps[t->getPropi(ITexture::ID)]);

		}
		setTextureUnit(index);
		updateShader(mm);
	}
	else
		PropertyManager::updateProp(pgTextureProps, name.ToStdString(), ITextureSampler::Attribs, (AttributeValues *)ts);
}


void DlgMaterials::updateTexture(ITexture *tex) {

	std::shared_ptr<Material> &mm = getModelMaterial();

	const int index = pgTextureProps->GetPropertyValueAsLong(wxT("Texture Unit"));

	ITexture *texture = mm->getTexture(index);

	if (texture == tex) {
		updateTextures(mm,index);
	}

}

void DlgMaterials::updateActiveTexture() {

	const int index = pgTextureProps->GetPropertyValueAsLong(wxT("Texture Unit"));
	std::shared_ptr<Material> &mm = getModelMaterial();

	updateTextures(mm,index);

}

void DlgMaterials::updateTextures(std::shared_ptr<Material> &mm, int index) {

	updateTextureList();
	ITexture *texture;
	for(int i = 0; i < 2; i++) { 
		for(int j = 0 ; j < 4 ; j++) {
			if (mm)
				texture = mm->getTexture(i * 4 + j);
			else
				texture = NULL;
			if (texture != NULL)
				imagesGrid[i * 4 + j]->setBitmap(DlgTextureLib::Instance()->m_Bitmaps[texture->getPropi(ITexture::ID)]);
//				imagesGrid[i * 4 + j]->setBitmap(texture->getBitmap());
			else
				imagesGrid[i*4+j]->setBitmap(m_EmptyBitmap);
		}
	}
	setTextureUnit(index);
	gridTextures->Refresh(true);
}



void DlgMaterials::updateTextureList() {

	int i,num = RESOURCEMANAGER->getNumTextures();

	pgTextureProps->ClearSelection();
	textureLabels.RemoveAt(0,textureLabels.GetCount());
	textureLabels.Add(wxT("No Texture"),-1);

	for (i = 0; i < num; i++) {
		textureLabels.Add(wxString(RESOURCEMANAGER->getTexture(i)->getLabel().c_str()),i);
	}
	m_pgPropTextureList->SetChoices(textureLabels);
}




void DlgMaterials::OnprocessClickGrid(wxGridEvent &e) {

	wxString s;

	int row = e.GetRow();
	int col = e.GetCol();

	setTextureUnit(row*4+col);
	
}


/* ----------------------------------------------------------------------

	Materials Stuff

-----------------------------------------------------------------------*/



void 
DlgMaterials::setupColorPanel(wxSizer *siz, wxWindow *parent) {

	wxColour col;


	pgMaterial = new wxPropertyGridManager(parent, DLG_MI_PGMAT,
				wxDefaultPosition, wxDefaultSize,
				// These and other similar styles are automatically
				// passed to the embedded wxPropertyGrid.
				wxPG_BOLD_MODIFIED|wxPG_SPLITTER_AUTO_CENTER|
				// Plus defaults.
				wxPGMAN_DEFAULT_STYLE
           );

	pgMaterial->AddPage(wxT("Colours"));

	std::vector<std::string> order = {"DIFFUSE", "AMBIENT", "EMISSION", "SPECULAR", "SHININESS" };
	PropertyManager::createOrderedGrid(pgMaterial, ColorMaterial::Attribs, order);

	pgMaterial->SetSplitterLeft(true);

	siz->Add(pgMaterial,1,wxEXPAND);
	updateColors(getModelMaterial()); // what is default material?
}


void 
DlgMaterials::resetColorPanel() {

	pgMaterial->Clear();
	pgMaterial->AddPage(wxT("Colours"));
	std::vector<std::string> order = { "DIFFUSE", "AMBIENT", "EMISSION", "SPECULAR", "SHININESS" };
	PropertyManager::createOrderedGrid(pgMaterial, ColorMaterial::Attribs, order);
}


void 
DlgMaterials::OnProcessColorChange( wxPropertyGridEvent& e){

	ColorMaterial *cm;
	wxString& name = e.GetPropertyName();
	wxPGProperty *topProp = e.GetProperty()->GetParent();
	const wxString& topName = e.GetProperty()->GetParent()->GetName();
	if (topName != "<Root>")
		name = topName;

	cm = &getModelMaterial()->getColor();
	PropertyManager::updateProp(pgMaterial, name.ToStdString(), ColorMaterial::Attribs, (AttributeValues *)cm);
}




void 
DlgMaterials::updateColors(std::shared_ptr<Material> &mm) {
	if (mm)
		PropertyManager::updateGrid(pgMaterial, ColorMaterial::Attribs, (AttributeValues *)mm.get());
}



/* ----------------------------------------------------------------------

	Shaders Stuff

-----------------------------------------------------------------------*/



void 
DlgMaterials::setupShaderPanel(wxSizer *siz, wxWindow *parent) {

	wxStaticBox *shaderSB = new wxStaticBox(parent,-1,wxT("Shaders"));
	wxSizer *sizerS = new wxStaticBoxSizer(shaderSB,wxVERTICAL);

	// TOP: Shader list and buttons
	wxBoxSizer *sizH1 = new wxBoxSizer(wxHORIZONTAL);
	wxStaticText *stg1 =  new wxStaticText(parent,-1,wxT("Shader: "));
	shaderList = new wxComboBox(parent,DLG_SHADER_COMBO,wxT(""),wxDefaultPosition,wxDefaultSize,0,NULL,wxCB_READONLY );

	sizH1->Add(stg1, 5, wxGROW|wxHORIZONTAL,5);
	sizH1->Add(shaderList, 5, wxGROW|wxHORIZONTAL,5);
	sizerS->Add(sizH1,1, wxEXPAND|wxALL|wxALIGN_CENTER,5);


	// Buttons
	wxSizer *sizerS_SB = new wxBoxSizer(wxHORIZONTAL);
		
	m_cbUseShader = new wxCheckBox(parent,DLG_SHADER_USE,wxT("Use Shaders"));
	m_cbUseShader->SetValue(0);

	sizerS_SB->Add(m_cbUseShader,0,wxALIGN_CENTER,5);
	sizerS->Add(sizerS_SB,0,wxGROW|wxALL|wxALIGN_CENTER,5);

	/* Uniforms */
	wxStaticBox *shaderU = new wxStaticBox(parent,-1,wxT("Uniforms"));
	wxSizer *sizerU = new wxStaticBoxSizer(shaderU,wxVERTICAL);

	pgShaderUniforms = new wxPropertyGridManager(parent, DLG_SHADER_UNIFORMS,
				wxDefaultPosition, wxDefaultSize,
				// These and other similar styles are automatically
				// passed to the embedded wxPropertyGrid.
				wxPG_BOLD_MODIFIED|wxPG_SPLITTER_AUTO_CENTER|
				// Plus defaults.
				wxPGMAN_DEFAULT_STYLE
           );

	pgShaderUniforms->AddPage(wxT("Shader Uniforms"));

	sizerU->Add(pgShaderUniforms,1,wxGROW | wxALL | wxALIGN_CENTER,5);

	siz->Add(sizerS,0,wxGROW | wxALL | wxALIGN_CENTER,5);
	siz->Add(sizerU,1,wxGROW | wxALL | wxALIGN_CENTER,5);

	updateShader(getModelMaterial());

}

void DlgMaterials::OnShaderListSelect(wxCommandEvent& event){

	std::shared_ptr<Material> &mm = getModelMaterial();

	int sel;
	wxString selString;

	sel = event.GetSelection();
	selString = shaderList->GetString(sel);

	if (sel > 0) {
		mm->attachProgram(std::string(selString.mb_str()));
	}
	else
		mm->attachProgram("");

	updateUniforms(mm);
}


void DlgMaterials::updateShader(std::shared_ptr<Material> &m){

	if (!m)
		return;

	// Update Shader List
	std::vector<std::string> *names = RESOURCEMANAGER->getProgramNames();
	int num = names->size();

	// shaderList should only be updated when a new shader is added or when a project is loaded
	shaderList->Clear();
	shaderList->Append(wxT("None"));

	for(int i = 0; i < num; i++)  {
		wxString s;
		s << i;
		shaderList->Append(wxString(names->at(i).c_str()));
	}
	delete names;

	// Clear Panel
	pgShaderUniforms->ClearPage(0);

	std::string progName = m->getProgramName();

	if (progName == "") {

		shaderList->SetSelection(0);
	}
	else {

		shaderList->SetStringSelection(wxString(progName.c_str()));

		// camList should only be updated when a new camera is added
		m_pgCamList.Clear();
		std::vector<std::string> camNames;
		RENDERMANAGER->getCameraNames(&camNames);
		for (unsigned int n = 0; n < camNames.size(); n++) {
		
			m_pgCamList.Add(wxString(camNames[n].c_str()),n);
		}

		// lightlist should only be updated when a new light is added.
		m_pgLightList.Clear();

		std::vector<std::string> lightNames;
		RENDERMANAGER->getLightNames(&lightNames);
		for ( unsigned int n = 0; n < lightNames.size(); n++) {
		
			m_pgLightList.Add(wxString(lightNames[n].c_str()),n);
		}

		m_pgTextureList.Clear();
		std::vector<std::string> textureNames;
		std::vector<std::string>::iterator it;
		m->getTextureNames(&textureNames);
		if (textureNames.size() != 0) {
			std::vector<unsigned int> textureUnits;
			m->getTextureUnits(&textureUnits);
			std::vector<unsigned int>::iterator itUnits;
			it = textureNames.begin();
			for ( itUnits = textureUnits.begin(); it != textureNames.end(); it++, itUnits++) {
			
				m_pgTextureList.Add(wxString((*it).c_str()),(int)(*itUnits));
			}
		}

		m_pgTextUnitsList.Clear();
		std::vector<unsigned int> textureUnits;
		m->getTextureUnits(&textureUnits);
		if (textureUnits.size()) {
			std::vector<unsigned int>::iterator itUnits;
			itUnits = textureUnits.begin();
			for ( ; itUnits != textureUnits.end(); itUnits++) {
				wxString s;
				s.Printf(wxT("%d"),(*itUnits));
				m_pgTextUnitsList.Add(s,(*itUnits));
			}
		}
		updateShaderAux(m);
	}
}

void DlgMaterials::updateShaderAux(std::shared_ptr<Material> &m) {

	if (NULL != m->getProgram()) {
		m_cbUseShader->SetValue(m->isShaderEnabled());
		updateUniforms(m);
	}
}


void DlgMaterials::addUniform(wxPGProperty *pdefaultBlock, ProgramValue  &u, int showGlobal) {

	int edit = strncmp("gl_", u.getName().c_str(), 3);
	if ((!showGlobal) && (edit == 0))
		return;

	wxPGProperty *pid, *pid2;
	pid = pgShaderUniforms->AppendIn(pdefaultBlock, new wxPGProperty(wxString((u.getName().c_str())), wxPG_LABEL));
	//if (u.getLoc() == -1)
	//	pgShaderUniforms->DisableProperty(pid);
	//else
	//	pgShaderUniforms->LimitPropertyEditing(pid);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Data Type"), wxPG_LABEL, wxString(Enums::DataTypeToString[u.getValueType()].c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	Enums::DataType vt = u.getValueType();
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Type"), wxPG_LABEL, wxString(u.getType().c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Context"), wxPG_LABEL, wxString(u.getContext().c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Component"), wxPG_LABEL, wxString(u.getValueOf().c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	std::string s = u.getType();
	if ((s == "MATERIAL_TEXTURE" || s == "IMAGE_TEXTURE" || s == "BUFFER_BINDING" || s == "TEXTURE_SAMPLER" || s == "TEXTURE_BINDING") ||
		(s == "LIGHT" && u.getContext() == "CURRENT")) {
		pid2 = pgShaderUniforms->AppendIn(pid, new wxIntProperty(wxT("id"), wxPG_LABEL, u.getId()));
		pgShaderUniforms->DisableProperty(pid2);
	}
}

void 
DlgMaterials::addBlockUniform(wxPGProperty * pblock, ProgramBlockValue & u, int showGlobal) {

	wxPGProperty *pid, *pid2;
	pid = pgShaderUniforms->AppendIn(pblock, new wxPGProperty(wxString(u.getName().c_str()), wxPG_LABEL));
	//if (u.getLoc() == -1)
	//	pgShaderUniforms->DisableProperty(pid);
	//else
	//	pgShaderUniforms->LimitPropertyEditing(pid);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Data Type"), wxPG_LABEL, wxString(Enums::DataTypeToString[u.getValueType()].c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	Enums::DataType vt = u.getValueType();
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Type"), wxPG_LABEL, wxString(u.getType().c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Context"), wxPG_LABEL, wxString(u.getContext().c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Component"), wxPG_LABEL, wxString(u.getValueOf().c_str())));
	pgShaderUniforms->DisableProperty(pid2);

	std::string s = u.getType();
	if ((s == "MATERIAL_TEXTURE" || s == "IMAGE_TEXTURE" || s == "BUFFER_BINDING" || s == "TEXTURE_SAMPLER" || s == "TEXTURE_BINDING") ||
		(s == "LIGHT" && u.getContext() == "CURRENT")) {
		pid2 = pgShaderUniforms->AppendIn(pid, new wxIntProperty(wxT("id"), wxPG_LABEL, u.getId()));
		pgShaderUniforms->DisableProperty(pid2);
	}
}


void
DlgMaterials::auxSetVec4(wxPGProperty *pid, wxPGProperty *pid2, int edit, float *f) 
{
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("fvalues"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("x"),wxPG_LABEL,f[0]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("y"),wxPG_LABEL,f[1]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("z"),wxPG_LABEL,f[2]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("w"),wxPG_LABEL,f[3]));
		pgShaderUniforms->Collapse(pid2);
		if (!edit)
			pgShaderUniforms->DisableProperty(pid2); // NO EDITS
}


void
DlgMaterials::auxSetMat4(wxPGProperty  *pid, wxPGProperty  *pid2, int edit, float *f) 
{
	pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("fvalues"),wxPG_LABEL));
	pgShaderUniforms->LimitPropertyEditing(pid2);
	if (!edit)
		pgShaderUniforms->DisableProperty(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 1"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m11"),wxPG_LABEL,f[0]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m12"),wxPG_LABEL,f[1]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m13"),wxPG_LABEL,f[2]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m14"),wxPG_LABEL,f[3]));
		pgShaderUniforms->Collapse(pid2);
		pgShaderUniforms->LimitPropertyEditing(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 2"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m21"),wxPG_LABEL,f[4]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m22"),wxPG_LABEL,f[5]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m23"),wxPG_LABEL,f[6]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m24"),wxPG_LABEL,f[7]));
		pgShaderUniforms->LimitPropertyEditing(pid2);
		pgShaderUniforms->Collapse(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 3"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m31"),wxPG_LABEL,f[8]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m32"),wxPG_LABEL,f[9]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m33"),wxPG_LABEL,f[10]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m34"),wxPG_LABEL,f[11]));
		pgShaderUniforms->LimitPropertyEditing(pid2);
		pgShaderUniforms->Collapse(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 4"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m41"),wxPG_LABEL,f[12]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m42"),wxPG_LABEL,f[13]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m43"),wxPG_LABEL,f[14]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m44"),wxPG_LABEL,f[15]));
		pgShaderUniforms->LimitPropertyEditing(pid2);
		pgShaderUniforms->Collapse(pid2);

}


void
DlgMaterials::auxSetMat3(wxPGProperty *pid, wxPGProperty *pid2, int edit, float *f) 
{
	pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("fvalues"),wxPG_LABEL));
	pgShaderUniforms->LimitPropertyEditing(pid2);
	if (!edit)
		pgShaderUniforms->DisableProperty(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 1"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m11"),wxPG_LABEL,f[0]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m12"),wxPG_LABEL,f[1]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m13"),wxPG_LABEL,f[2]));
		pgShaderUniforms->Collapse(pid2);
		pgShaderUniforms->LimitPropertyEditing(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 2"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m21"),wxPG_LABEL,f[4]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m22"),wxPG_LABEL,f[5]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m23"),wxPG_LABEL,f[6]));
		pgShaderUniforms->LimitPropertyEditing(pid2);
		pgShaderUniforms->Collapse(pid2);
	pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 3"), wxPG_LABEL, wxT("<composed>")));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m31"),wxPG_LABEL,f[8]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m32"),wxPG_LABEL,f[9]));
		pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m33"),wxPG_LABEL,f[10]));
		pgShaderUniforms->LimitPropertyEditing(pid2);
		pgShaderUniforms->Collapse(pid2);
}


void DlgMaterials::OnProcessUseShader(wxCommandEvent& event){

	std::shared_ptr<Material> &m = getModelMaterial();

	m->enableShader(event.IsChecked());
}


void DlgMaterials::updateUniforms(std::shared_ptr<Material> &m) {

	//m->checkProgramValuesAndUniforms();
	pgShaderUniforms->ClearPage(0);

	if (m->getProgram() == NULL) {
		pgShaderUniforms->Refresh();
		return;
	}

	wxPGProperty *pid = new wxPGProperty(wxT("Default Block"), wxPG_LABEL);
	pgShaderUniforms->Append(pid);
	std::map<std::string, nau::material::ProgramValue> progValues;
	progValues = m->getProgramValues();
	ProgramValue pv;
	for (auto progVal: progValues) {
		pv = progVal.second;
		addUniform(pid, pv, false);

	}
	std::map<std::pair<std::string, std::string>, nau::material::ProgramBlockValue> progBlockValues;
	progBlockValues = m->getProgramBlockValues();
	ProgramBlockValue pbv;
	std::string blockName = "";
	for (auto progVal: progBlockValues) {
		pbv = progVal.second;
		if (progVal.first.first != blockName) {
			blockName = progVal.first.first;
			pid = new wxPGProperty(wxString(blockName.c_str()), wxPG_LABEL);
			pgShaderUniforms->Append(pid);
		}
		addBlockUniform(pid, pbv, false);

	}
	pgShaderUniforms->Refresh();
}



// this has no effect since m_Values is updated when getting the uniform value
void 
DlgMaterials::OnProcessShaderUpdateUniforms( wxPropertyGridEvent& e) {

	const wxString& name = e.GetPropertyName();
	unsigned int dotLocation = name.find_first_of(wxT("."),0);
	std::string topProp = std::string(name.substr(0,dotLocation).mb_str());
	std::string prop = std::string(name.substr(dotLocation+1,name.size()-dotLocation-1).mb_str());

	std::shared_ptr<Material> &m = getModelMaterial();

	if ("Type" == prop) {

	} 
	else if ("Semantic Class" == prop) {
	
	}
	else if ("Context" == prop) {

	}
	else if ("Semantics" == prop) {
		
	}
	else if ("ivalue" == prop) {
		int i = e.GetPropertyValue().GetInteger();
		m->setValueOfUniform(topProp, &i);
	}
	else if ("fvalue" == prop) {
		float f = e.GetPropertyValue().GetDouble();
		m->setValueOfUniform(topProp, &f);
	}
}


//---------------------------- OLD CODE CUT HERE --------------------------------------------------------

//void DlgMaterials::updateUniforms(Material *m) {
//
//	m->clearUniformValues();
//	pgShaderUniforms->ClearPage(0);
//
//	if (m->getProgram() == NULL) {
//		pgShaderUniforms->Refresh();
//		return;
//	}
//
//	GLProgram *p = (GLProgram *)m->getProgram();
//
//	std::map<std::string, nau::material::ProgramValue> progValues, uniformValues;
//	std::map<std::string, nau::material::ProgramValue>::iterator progValuesIter;
//	progValues = m->getProgramValues();
//
//	GLUniform u;
//	std::string s;
//	int uni = p->getNumberOfUniforms();
//	p->updateUniforms();
//	m->clearUniformValues();
//	for (int i = 0; i < uni; i++) {
//		// get uniform from program
//		u = p->getUniform(i);
//		// add uniform (material class will take care of special cases)
//		//s = "DATA(" + u.getName() + "," + u.getProgramValueType() + ")";
//		m->addProgramValue(u.getName(), ProgramValue(u.getName(), "DATA", u.getStringSimpleType(), "", 0, false));
//		m->setValueOfUniform(u.getName(), (float *)u.getValues());
//	}
//
//	for (int i = 0; i < 8; i++)
//		samplerUnits[i] = 0;
//
//	std::vector<std::string> *validProgValueNames = m->getValidProgramValueNames();
//	std::vector<std::string>::iterator validNamesIter;
//	//progValues = m->getProgramValues();
//	validNamesIter = validProgValueNames->begin();
//	ProgramValue pv;
//	for (; validNamesIter != validProgValueNames->end(); validNamesIter++) {
//		pv = *m->getProgramValue(*validNamesIter);
//		if (p->findUniform(*validNamesIter) != -1) {
//			//	if (uniformValues.count(progValuesIter->first) == 0) {
//			//		addUniform((*progValuesIter).second,m_cbShowGlobalU->GetValue());
//			addUniform(pv, false);
//			if (pv.getValueType() == Enums::SAMPLER && pv.getContext() == "TEXTURE") {
//				int *ip = (int *)pv.getValues();
//				samplerUnits[ip[0]]++;
//			}
//			//	}
//		}
//	}
//	uniformValues = m->getUniformValues();
//	progValuesIter = uniformValues.begin();
//	for (; progValuesIter != uniformValues.end(); progValuesIter++) {
//		pv = (*progValuesIter).second;
//		addUniform(pv, false);
//
//	}
//	pgShaderUniforms->Refresh();
//}


//void DlgMaterials::addUniform(ProgramValue  &u, int showGlobal) {

	//const wxChar* vec4LightComp[] = {wxT("POSITION"), wxT("DIRECTION"), wxT("NORMALIZED_DIRECTION"), wxT("COLOR"), wxT("AMBIENT"), NULL};
	//const long vec4LightCompInd[] = {Light::POSITION, Light::DIRECTION, Light::NORMALIZED_DIRECTION, Light::COLOR,
	//			Light::AMBIENT};

	//const wxChar* floatLightComp[] = {wxT("SPOT_EXPONENT"), wxT("SPOT_CUTOFF"), wxT("CONSTANT_ATT"), wxT("LINEAR_ATT"), wxT("QUADRATIC_ATT"), NULL};
	//const long floatLightCompInd[] = {Light::SPOT_EXPONENT, Light::SPOT_CUTOFF, Light::CONSTANT_ATT, Light::LINEAR_ATT,
	//			Light::QUADRATIC_ATT};

	//const wxChar* boolLightComp[] = {wxT("ENABLED"), NULL};
	//const long boolLightCompInd[] = {Light::ENABLED};

	////const wxChar* enumLightComp[] = {wxT("TYPE"), NULL};
	////const long enumLightCompInd[] = {Light::TYPE};

	//int edit = strncmp("gl_", u.getName().c_str(), 3);
	//if ((!showGlobal) && (edit == 0))
	//	return;

	//wxPGProperty *pid, *pid2;
	//const wxChar* units[] = {wxT("0"),wxT("1"),wxT("2"),wxT("3"),wxT("4"),wxT("5"),wxT("6"),wxT("7"),NULL};
	//const long unitsInd[] = {0,1,2,3,4,5,6,7};

	//const wxChar* vec4CamComp[] = {wxT("POSITION"), wxT("VIEW"),  
	//									wxT("UP"), wxT("NORMALIZED_UP"), 
	//									wxT("NORMALIZED_RIGHT"), wxT("LOOK_AT_POINT"), NULL};
	//const long vec4CamCompInd[] = {Camera::POSITION, Camera::VIEW_VEC, 
	//									Camera::UP_VEC, Camera::NORMALIZED_UP_VEC,
	//									Camera::NORMALIZED_RIGHT_VEC, Camera::LOOK_AT_POINT};


	//const wxChar* mat4CamComp[] = {wxT("VIEW_MATRIX"), wxT("PROJECTION_MATRIX"), 
	//									wxT("VIEW_INVERSE_MATRIX"), wxT("PROJECTION_VIEW_MATRIX"), wxT("TS05_PVM_MATRIX"),NULL};
	//const long mat4CamCompInd[] = {Camera::VIEW_MATRIX, Camera::PROJECTION_MATRIX, 
	//									Camera::VIEW_INVERSE_MATRIX, Camera::PROJECTION_VIEW_MATRIX, Camera::TS05_PVM_MATRIX};


	//const wxChar* floatCamComp[] = {wxT("FOV"), wxT("NEARP"), wxT("FARP"), wxT("LEFT"), wxT("RIGHT"), wxT("TOP"), wxT("BOTTOM"),NULL};
	//const long floatCamCompInd[] = {Camera::FOV, Camera::NEARP, 
	//									Camera::FARP, Camera::LEFT, Camera::RIGHT, 
	//									Camera::TOP, Camera::BOTTOM};

	//const wxChar* intTextureComp[] = {wxT("WIDTH"), wxT("HEIGHT"), wxT("DEPTH"), NULL};
	//const long intTextureCompInd[] = {ITexture::WIDTH, ITexture::HEIGHT, ITexture::DEPTH};

	//const wxChar* vec3SemanticClass[] = {wxT("Camera"), wxT("Light"), wxT("Data"), NULL};
	//const long vec3SemanticClassInd[] = {ProgramValue::CAMERA, ProgramValue::LIGHT, ProgramValue::DATA};

	//const wxChar* mat4SemanticClass[] = {wxT("Camera"), wxT("Data"), NULL};
	//const long mat4SemanticClassInd[] = {ProgramValue::CAMERA, ProgramValue::DATA};

	//const wxChar* intSemanticClass[] = {wxT("Light"), wxT("Data"), NULL};
	//const long intSemanticClassInd[] = {ProgramValue::TEXTURE, ProgramValue::LIGHT, ProgramValue::DATA};

	//const wxChar* vec4ColorComp[] = {wxT("AMBIENT"),wxT("SPECULAR"),wxT("EMISSION"),wxT("DIFFUSE"), NULL};
	//const long vec4ColorCompInd[] = {ColorMaterial::AMBIENT,
	//							  ColorMaterial::SPECULAR,
	//							  ColorMaterial::EMISSION,
	//							  ColorMaterial::DIFFUSE};

	//const wxChar* floatColorComp[] = {wxT("SHININESS"), NULL};
	//const long floatColorCompInd[] = {ColorMaterial::SHININESS};

	//const wxChar* mat4MatrixComp[] = {wxT("PROJECTION"),
	//								wxT("MODEL"),
	//								wxT("VIEW"),
	//								wxT("TEXTURE"),
	//								wxT("VIEW_MODEL"),
	//								wxT("PROJECTION_VIEW_MODEL"),
	//								wxT("PROJECTION_VIEW"),
	//								wxT("TS05_PVM"), NULL};

	//const long mat4MatrixCompInd[] = {IRenderer::PROJECTION,
	//			IRenderer::MODEL,
	//			IRenderer::VIEW,
	//			IRenderer::TEXTURE,
	//			IRenderer::VIEW_MODEL,
	//			IRenderer::PROJECTION_VIEW_MODEL,
	//			IRenderer::PROJECTION_VIEW,
	//			IRenderer::TS05_PVM};

	//const wxChar* mat3MatrixComp[] = {wxT("NORMAL"), NULL};
	//const long mat3MatrixCompInd[] = {IRenderer::NORMAL};

	//pid = pgShaderUniforms->Append(new wxPGProperty(wxString((u.getName().c_str())), wxPG_LABEL));
	//if (u.getLoc() == -1)
	//	pgShaderUniforms->DisableProperty(pid);
	//else
	//	pgShaderUniforms->LimitPropertyEditing(pid);
	//pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Data Type"), wxPG_LABEL, wxString(Enums::DataTypeToString[u.getValueType()].c_str())));
	//pgShaderUniforms->DisableProperty(pid2);

	//Enums::DataType vt = u.getValueType();
	////	pid2 = pgShaderUniforms->AppendIn(pid,new wxStringProperty(wxT("Semantic Class"),wxPG_LABEL,wxString(ProgramValue::getSemanticTypeString(u.getSemanticType()).c_str())));	
	//pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Type"), wxPG_LABEL, wxString(u.getType().c_str())));
	//pgShaderUniforms->DisableProperty(pid2);

	//pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Context"), wxPG_LABEL, wxString(u.getContext().c_str())));
	//pgShaderUniforms->DisableProperty(pid2);

	//pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Component"), wxPG_LABEL, wxString(u.getValueOf().c_str())));
	//pgShaderUniforms->DisableProperty(pid2);

	//std::string s = u.getType();
	//if ((s == "TEXTURE" || s == "IMAGE_TEXTURE" || s == "MATERIAL_BUFFER" || s == "TEXTURE_SAMPLER" || s == "MATERIAL_TEXTURES") ||
	//	(s == "LIGHT" && u.getContext() == "CURRENT")) {
	//	pid2 = pgShaderUniforms->AppendIn(pid, new wxIntProperty(wxT("id"), wxPG_LABEL, u.getId()));
	//	pgShaderUniforms->DisableProperty(pid2);
	//}

	//	int *i;
	//	int auxI[4] = {0,0,0,0};
	//	float *f;
	//	float auxF[16] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
	//					  0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	//
	//	if (u.getContext() != "CURRENT") {
	//		i = (int *)u.getValues();
	//		f = (float *)u.getValues();
	//	}
	//	else {
	//					
	//
	//		if (u.getType() == "MATRIX" && u.getValueType() == Enums::MAT4) {
	//			pid2 = pgShaderUniforms->AppendIn(pid, new wxEnumProperty(wxT("Semantics"),wxPG_LABEL, mat4MatrixComp, mat4MatrixCompInd,u.getSemanticValueOf()));
	//		}
	//		else if (u.getType() == "MATRIX" && u.getValueType() == Enums::MAT3) {
	//			pid2 = pgShaderUniforms->AppendIn(pid, new wxEnumProperty(wxT("Semantics"),wxPG_LABEL, mat3MatrixComp, mat3MatrixCompInd,u.getSemanticValueOf()));
	//		}
	//		else if (u.getType() == "COLOR" && u.getValueType() == Enums::VEC4){
	//			pid2 = pgShaderUniforms->AppendIn(pid, new wxEnumProperty(wxT("Semantics"),wxPG_LABEL, vec4ColorComp, vec4ColorCompInd, u.getSemanticValueOf()));
	//		}
	//		else if (u.getType() == "COLOR" && u.getValueType() == Enums::FLOAT){
	//			pid2 = pgShaderUniforms->AppendIn(pid, new wxEnumProperty(wxT("Semantics"),wxPG_LABEL, floatColorComp, floatColorCompInd, u.getSemanticValueOf()));	
	//		}
	//
	//		i = auxI;
	//		f = auxF;
	//	}
	//
	////	if (ProgramValue::CAMERA == u.getSemanticType()) {
	//	if (u.getType() == "CAMERA") {
	//
	//			pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Camera"),wxPG_LABEL,m_pgCamList));
	//			pgShaderUniforms->SetPropertyValueString(pid2,wxString(u.getContext().c_str()));
	//
	//			switch(u.getValueType()) {
	//				case Enums::FLOAT:
	//					pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,floatCamComp,floatCamCompInd,u.getSemanticValueOf()));
	//					break;
	//				case Enums::VEC4:
	//					pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,vec4CamComp,vec4CamCompInd,u.getSemanticValueOf()));
	//					break;
	//				case Enums::MAT4:
	//					pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,mat4CamComp,mat4CamCompInd,u.getSemanticValueOf()));
	//					break;
	//			}
	//	}
	//	else if (u.getType() == "LIGHT" || (u.getType() == "LIGHT" && "CURRENT" == u.getContext())) {
	//
	//		if (u.getType() == "LIGHT") {
	//			pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Light"),wxPG_LABEL,m_pgLightList));
	//			pgShaderUniforms->SetPropertyValueString(pid2,wxString(u.getContext().c_str()));
	//		}
	//		else {
	//			pid2 = pgShaderUniforms->AppendIn(pid,new wxIntProperty(wxT("ID"),wxPG_LABEL, u.getId()));
	//		}
	//		
	//		switch(u.getValueType()) {
	//			
	//			case Enums::VEC4:
	//				pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,vec4LightComp,vec4LightCompInd,u.getSemanticValueOf()));
	//				break;
	//
	//			case Enums::FLOAT:
	//				pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,floatLightComp,floatLightCompInd,u.getSemanticValueOf()));
	//				break;
	//
	//			case Enums::BOOL:
	//				pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,boolLightComp,boolLightCompInd,u.getSemanticValueOf()));
	//				break;
	//
	//			//case Enums::ENUM:
	//			//	pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,enumLightComp,enumLightCompInd,u.getSemanticValueOf()));
	//			//	break;
	//		}
	//	}
	//	else if (u.getType() == "TEXTURE" || (u.getType() == "TEXTURE" && "CURRENT" == u.getContext())) {
	//
	//		if (u.getType() == "TEXTURE") {
	//			pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("ITexture"),wxPG_LABEL,textureLabels/*m_pgTextureList*/));
	//			pgShaderUniforms->SetPropertyValueString(pid2,wxString(u.getContext().c_str()));
	//		}
	//					
	//		switch (u.getValueType()) {
	//
	//			case Enums::SAMPLER:
	//				pid2 = pgShaderUniforms->AppendIn(pid,new wxStringProperty(wxT("Semantics"),wxPG_LABEL,wxT("UNIT")));
	//				pgShaderUniforms->DisableProperty(pid2);
	//				pid2 = pgShaderUniforms->AppendIn(pid, new wxEnumProperty(wxT("ivalue"), wxPG_LABEL, m_pgTextUnitsList));
	//				pgShaderUniforms->SetPropertyValue(pid2, u.getId());
	//				break;
	//			case Enums::INT:
	//				pid2 = pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("Semantics"),wxPG_LABEL,intTextureComp,intTextureCompInd,u.getSemanticValueOf()));
	//				break;
	//		}
	//	}

	//else if (ProgramValue::DATA == u.getSemanticType()) {
	//		switch(u.getSemanticValueOf()) {
	//
	//			case ProgramValue::USERDATA: {
	//				
	//				switch(u.getValueType()) {
	//					
	//					case Enums::FLOAT:
	//						pgShaderUniforms->AppendIn(pid,new wxFloatProperty(wxT("fvalue"),wxPG_LABEL,f[0]));
	//						if (!edit)
	//							pgShaderUniforms->DisableProperty(pid2);
	//						break;
	//					case Enums::VEC2:
	//						pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("fvalues"),wxPG_LABEL));
	//						pgShaderUniforms->LimitPropertyEditing(pid2);
	//						if (!edit)
	//							pgShaderUniforms->DisableProperty(pid2);
	//						pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("v[0]"),wxPG_LABEL,f[0]));
	//						pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("v[1]"),wxPG_LABEL,f[1]));
	//						break;
	//					case Enums::VEC3:
	//						pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("fvalues"),wxPG_LABEL));
	//						pgShaderUniforms->LimitPropertyEditing(pid2);
	//						if (!edit)
	//							pgShaderUniforms->DisableProperty(pid2);
	//						pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("v[0]"),wxPG_LABEL,f[0]));
	//						pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("v[1]"),wxPG_LABEL,f[1]));
	//						pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("v[2]"),wxPG_LABEL,f[2]));
	//						break;
	//					case Enums::VEC4:
	//						auxSetVec4(pid,pid2,edit,f);
	//						break;

	//					case Enums::INT: 
	//					case Enums::BOOL:
	//						pgShaderUniforms->AppendIn(pid,new wxIntProperty(wxT("ivalue"),wxPG_LABEL,i[0]));
	//						break;
	//					case Enums::IVEC2:
	//					case Enums::BVEC2:
	//						pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("ivalues"),wxPG_LABEL));
	//						pgShaderUniforms->LimitPropertyEditing(pid2);
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[0]"),wxPG_LABEL,i[0]));
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[1]"),wxPG_LABEL,i[1]));
	//						break;
	//					case Enums::IVEC3:
	//					case Enums::BVEC3:
	//						pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("ivalues"),wxPG_LABEL));
	//						pgShaderUniforms->LimitPropertyEditing(pid2);
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[0]"),wxPG_LABEL,i[0]));
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[1]"),wxPG_LABEL,i[1]));
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[2]"),wxPG_LABEL,i[2]));
	//						break;
	//					case Enums::IVEC4:
	//					case Enums::BVEC4:
	//						pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("ivalues"),wxPG_LABEL));
	//						pgShaderUniforms->LimitPropertyEditing(pid2);
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[0]"),wxPG_LABEL,i[0]));
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[1]"),wxPG_LABEL,i[1]));
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[2]"),wxPG_LABEL,i[2]));
	//						pgShaderUniforms->AppendIn(pid2,new wxIntProperty(wxT("v[3]"),wxPG_LABEL,i[3]));
	//						break;

	//						case Enums::SAMPLER:
	//						pgShaderUniforms->AppendIn(pid,new wxEnumProperty(wxT("texture unit"),wxPG_LABEL,m_pgTextUnitsList,i[0]));
	//						break;

	//					case Enums::MAT4:
	//						auxSetMat4(pid,pid2,edit,f);
	//						break;
	//					case Enums::MAT3:
	//						auxSetMat3(pid,pid2,edit,f);
	//						break;
	//					case Enums::MAT2:
	//						pid2 = pgShaderUniforms->AppendIn(pid,new wxPGProperty(wxT("fvalues"),wxPG_LABEL));
	//						pgShaderUniforms->LimitPropertyEditing(pid2);
	//						if (!edit)
	//							pgShaderUniforms->DisableProperty(pid2);
	//						pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 1"), wxPG_LABEL, wxT("<composed>")));
	//							pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m11"),wxPG_LABEL,f[0]));
	//							pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m12"),wxPG_LABEL,f[1]));
	//							pgShaderUniforms->LimitPropertyEditing(pid2);
	//							pgShaderUniforms->Collapse(pid2);
	//						pid2 = pgShaderUniforms->AppendIn(pid, new wxStringProperty(wxT("Row 2"), wxPG_LABEL, wxT("<composed>")));
	//							pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m21"),wxPG_LABEL,f[4]));
	//							pgShaderUniforms->AppendIn(pid2,new wxFloatProperty(wxT("m22"),wxPG_LABEL,f[5]));
	//							pgShaderUniforms->LimitPropertyEditing(pid2);
	//							pgShaderUniforms->Collapse(pid2);
	//						break;
	//				}
	//			}
	//		}	
	//}
//}
