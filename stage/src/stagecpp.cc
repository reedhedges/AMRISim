// Thin-as-possible C Wrappers for C++ worldfile calls, using a single static worldfile.
// This is a hacky use of the old C++ worldfile code.
// $Id$

#include "stage_internal.h"
#include "gui.h"
#include "worldfile.hh"


static CWorldFile wf;

int wf_property_exists( int section, const char* token )
{
  return( wf.GetProperty( section, token ) > -1 );
}

// read wrappers
int wf_read_int( int section, const char* token, int def )
{
  return wf.ReadInt( section, token, def );
}

double wf_read_float( int section, const char* token, double def )
{
  return wf.ReadFloat( section, token, def );
}

double wf_read_length( int section, const char* token, double def )
{
  return wf.ReadLength( section, token, def );
}

double wf_read_angle( int section, const char* token, double def )
{
  return wf.ReadAngle( section, token, def );
}

const char* wf_read_string( int section, const char* token, const char* def )
{
  return wf.ReadString( section, token, def );
}

const char* wf_read_tuple_string( int section, const char* token, int index, const char* def )
{
  return wf.ReadTupleString( section, token, index, def );
}

double wf_read_tuple_float( int section, const char* token, int index, double def )
{
  return wf.ReadTupleFloat( section, token, index, def );
}

double wf_read_tuple_length( int section, const char* token, int index, double def )
{
  return wf.ReadTupleLength( section, token, index, def );
}

double wf_read_tuple_angle( int section, const char* token, int index, double def )
{
  return wf.ReadTupleAngle( section, token, index, def );
}

// write wrappers

void wf_write_int( int section, const char* token, int value )
{
  wf.WriteInt( section, token, value );
}

void wf_write_float( int section, const char* token, double value )
{
  wf.WriteFloat( section, token, value );
}

void wf_write_length( int section, const char* token, double value )
{
  wf.WriteLength( section, token, value );
}

//void wf_write_angle( int section, const char* token, double value )
//{
//wf.WriteAngle( section, token, value );
//}

void wf_write_string( int section, const char* token, const char* value )
{
  wf.WriteString( section, token, value );
}

void wf_write_tuple_string( int section, const char* token, int index, const char* value )
{
  wf.WriteTupleString( section, token, index, value );
}

void wf_write_tuple_float( int section, const char* token, int index, double value )
{
  wf.WriteTupleFloat( section, token, index, value );
}

void wf_write_tuple_length( int section, const char* token, int index, double value )
{
  wf.WriteTupleLength( section, token, index, value );
}

void wf_write_tuple_angle( int section, const char* token, int index, double value )
{
  wf.WriteTupleAngle( section, token, index, value );
}

void wf_write_tuple_int(int section, const char* token, int index, int value)
{
  wf.WriteTupleInt(section, token, index, value);
}

void wf_save( void )
{
  wf.Save( NULL );
}

int wf_load( const char* path, int echo )
{
  return wf.Load( path, echo );
}

int wf_section_count( void )
{
  return wf.GetEntityCount();
}

const char* wf_get_section_type( int section )
{
  return wf.GetEntityType( section );
}


int wf_get_parent_section( int section )
{
  return wf.GetEntityParent( section );
}

const char* wf_get_filename( )
{
  return( (const char*)wf.filename );
}

int wf_lookup_section(const char *type)
{
  return wf.LookupEntity(type);
}


int wf_lookup_macro_def(const char *type)
{
  return wf.LookupMacro(type);
}

const char * wf_get_macro_name(int macro)
{
  return wf.GetMacroName(macro);
}

const char * wf_get_macro_section_name(int macro)
{
  return wf.GetMacroEntityName(macro);
}

int wf_get_macro_parent(int macro)
{
  return wf_lookup_macro_def(wf_get_macro_section_name(macro));
}

const char * wf_get_macro_base_type(int macro)
{
  /* go down the macro "inheritance" lineage to get to the base. */
  int id = macro;
  while(1)
  {
    const char *name = wf_get_macro_section_name(id);
    id = wf_lookup_macro_def(name);
    if(id == -1) return name; // wasn't a macro definition.
  }
}



const char *wf_get_macro_parent_name(int macro) {
  return wf_get_macro_section_name(macro);
}


int wf_create_entity(const char *type, int parentent)
{
  return wf.CreateEntity(type, parentent);
}

int wf_get_next_child_entity(int parent, int start)
{
  return wf.GetNextChildEntity(parent, start);
}

const char *wf_get_section_immediate_type(int section)
{
  return wf.GetEntityImmediateType(section);
}

void wf_set_entity_id_for_model_type(char *typestr, stg_id_t entity_id)
{
  wf.SetEntityIdForModelType(typestr, entity_id);
}

int wf_find_entity_id_for_model_type(char *typestr)
{
  return wf.FindEntityIdForModelType(typestr);
}



