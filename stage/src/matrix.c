
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1 // maybe get extensions like sincos()
#endif

#ifndef __USE_GNU
#define __USE_GNU 1 // maybe get extensions like sincos()
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h> // for memcpy(3)

#include "stage_internal.h"
#include "gui.h" // for layer definitions

//#define DEBUG 1
/*
#define PRINT_NUM_POLYS_RENDERED  10000
#define PRINT_NUM_POINTS_RENDERED 100000
*/

extern stg_rtk_fig_t* fig_debug_matrix;
extern int _render_matrix_deltas;

//stg_rtk_fig_t* fig_monitor_matrix = NULL;
//GList* doomed = NULL;
//GList* created = NULL;

extern stg_rtk_fig_t* fig_debug_matrix;

stg_cell_t* stg_cell_create( stg_cell_t* parent, double x, double y, double size )
{
  stg_cell_t* cell = (stg_cell_t*)calloc( sizeof(stg_cell_t), 1);
  
  cell->parent = parent;
  cell->data = NULL;
  cell->x = x;
  cell->y = y;
  cell->size = size;
 
  // store bounds for fast checking
  cell->xmin = x - size/2.0;
  cell->xmax = x + size/2.0;
  cell->ymin = y - size/2.0;
  cell->ymax = y + size/2.0;

  // taken care of by the calloc()
  //cell->fig = NULL;
    
  PRINT_DEBUG1("created matrix cell %p.\n", cell);

  return cell;
}

void stg_cell_delete( stg_cell_t* cell )
{

  PRINT_DEBUG1("deleting matrix cell %p.\n", cell);

  if( cell->fig )
    stg_rtk_fig_destroy( cell->fig );

  // free memory used internally by GSList, this ought not
  // free the actual items pointed to by the GSList (model/object).
  // (rh)
  if(cell->data)
    g_slist_free(cell->data);
  
  free( cell );
}

// Recursively free the quadtree of cells (rh)
void stg_cell_and_descendents_delete(stg_cell_t* cell)
{
   int i;
   if(!cell) return;
   for(i = 0; i < 4; i++)
     if(cell->children[i])
       stg_cell_and_descendents_delete(cell->children[i]);
   stg_cell_delete(cell);
}


void stg_cell_unrender( stg_cell_t* cell )
{
  if( cell->fig )
    stg_rtk_fig_destroy( cell->fig );
  cell->fig = NULL;
}    

void stg_cell_render( stg_cell_t* cell )
{
  
  cell->fig = stg_rtk_fig_create( fig_debug_matrix->canvas,
				  fig_debug_matrix,
				  STG_LAYER_MATRIX_TREE, "<debug.matrixcell>" );
  
  stg_rtk_fig_color_rgb32( cell->fig, 0x00AA00 );
  
  stg_rtk_fig_rectangle( cell->fig,
			 cell->x, cell->y, 0.0,
			 cell->size, cell->size, 0 );
}

/* void stg_cell_render_tree( stg_cell_t* cell ) */
/* { */
/*   //stg_cell_render( cell ); */

/*   if( cell->children[0] ) */
/*     { */
/*       int i; */
/*       for( i=0; i<4; i++ ) */
/* 	stg_cell_render_tree( cell->children[i] ); */
/*     } */
/* } */

void stg_cell_unrender_tree( stg_cell_t* cell )
{
  if( cell->children[0] )
    {
      int i;
      for( i=0; i<4; i++ )
	stg_cell_unrender_tree( cell->children[i] );
    }

  stg_cell_unrender( cell );
}


stg_matrix_t* stg_matrix_create( double ppm, double width, double height )
{
  double sz;
  stg_matrix_t* matrix = NULL;
  matrix = (stg_matrix_t*)calloc(1, sizeof(stg_matrix_t));
  assert(matrix);

  matrix->ppm = ppm; // base resolution

  // grow from the smallest cell size to find a root size that
  // encompasses the whole world
  sz = 1.0/ppm;
  while( sz < MAX(width,height) )
    sz *= 2.0;

  matrix->width = sz;//width;
  matrix->height = sz;//height;

  // create the root node of a quadtree
  matrix->root = stg_cell_create( NULL, 0.0, 0.0, sz );

  // hash table is indexed by object pointer and a list of the cells
  // each object is rendered into
  matrix->ptable = g_hash_table_new( g_direct_hash, g_direct_equal );

  matrix->locking_enabled = FALSE;

  return matrix;
}


// frees all memory allocated by the matrix; first the cells, then the
// cell array.
void stg_matrix_destroy( stg_matrix_t* matrix )
{
  PRINT_DEBUG1("destroying matrix %p", matrix);
  //printf("destroying matrix %p\n", matrix);
  if(matrix->root) // rh
  {
    stg_cell_and_descendents_delete(matrix->root);
    matrix->root = NULL;
  }
  if(matrix->ptable)
    g_hash_table_destroy( matrix->ptable );
  else
  {
    puts("Warning: matrix object hash table is NULL, not destroying. Memory corruption or bug (double matrix destroy). May leak memory.");
  }
  matrix->ptable = NULL;
  free( matrix );
}

// removes all pointers from every cell in the matrix
void stg_matrix_clear( stg_matrix_t* matrix )
{  
    // XXX todo
    /// @todo
    puts(" XXX stg_matrix_clear called but it's unimplemented. XXX ");
}

void stg_cell_print( stg_cell_t* cell, char* prefix )
{
  printf( "%s: cell %p at %.4f,%.4f size %.4f [%.4f:%.4f][%.4f:%.4f] children %p %p %p %p data %p\n",
      prefix,
      cell, 
      cell->x, 
      cell->y, 
      cell->size, 
      cell->xmin,
      cell->xmax,
      cell->ymin,
      cell->ymax,
      cell->children[0],
      cell->children[1],
      cell->children[2],
      cell->children[3],
      cell->data );  
}

void stg_cell_remove_object( stg_cell_t* cell, void* p )
{
  if(!cell)
  {
    printf("INTERNAL ERROR matrix cell is NULL. in stg_cell_remove_object(%p, %p)\n", cell, p);
    return;
  }
  if(!cell->data)
  {
    printf("INTERNAL ERROR matrix cell has NULL data. in stg_cell_remove_object(%p, %p)\n", cell, p);
    return;
  }

  cell->data = g_slist_remove( (GSList*)cell->data, p ); 

  // if the cell is empty and has a parent, we might be able to delete it
  if( cell->data == NULL && cell->parent )
  {
    // hop up in the tree
    cell = cell->parent;

    // while all children are empty
    while( cell && 
        !(cell->children[0]->children[0] || cell->children[0]->data ||
          cell->children[1]->children[0] || cell->children[1]->data ||
          cell->children[2]->children[0] || cell->children[2]->data ||
          cell->children[3]->children[0] || cell->children[3]->data) )
    {	      
      // detach siblings from parent and free them
      int i;
      for(i=0; i<4; i++ )
      {
        stg_cell_delete( cell->children[i] );
        cell->children[i] = NULL; 
      }

      cell = cell->parent;
    }
  }

}

// remove <object> from the matrix
void stg_matrix_remove_object( stg_matrix_t* matrix, void* object )
{
  // get the list of cells in which this object has been drawn
  GSList* list;
  GSList* it;

  gui_lock();
  
  list = g_hash_table_lookup( matrix->ptable, object );
  //printf("stg_matrix_remove_object: object %p has cell list %p.\n", object, list);

  // remove this object from each cell in the list      
  if(list)
  {
    for( it = list; it; it = it->next )
    {
      if(!object || !(it->data))
        printf( "Warning: removing object %p from cell %p\n", object, it->data );
      stg_cell_remove_object( (stg_cell_t*)it->data, object );
    }
    // now free the cell list
    g_slist_free( list );
  }
  else
  {
    //printf("warning, object %p (%s) has no matrix cell list in matrix hash table.\n", object, ( ((stg_model_t*)object)->token) );
  }


  // and remove the object from the hash table
  g_hash_table_remove( matrix->ptable, object );

  gui_unlock();
}


void stg_matrix_lines( stg_matrix_t* matrix, 
    stg_line_t* lines, int num_lines,
    void* object )
{
  size_t l;
  const double res = 1.0/matrix->ppm;
  gui_lock();


  for(l=0; l<num_lines; ++l )
  {
    // start these values, they will be used and changed as we iterate through cells
    // for the line to pass through
    double x1 = lines[l].x1;
    double y1 = lines[l].y1;
    double x2 = lines[l].x2;
    double y2 = lines[l].y2;

    // some values used in the loop but which remain constant for this line
    const double theta = atan2( y2-y1, x2-x1 );
    const double m = tan(theta); // line gradient 


    //int step = 0;

    stg_cell_t* cell = matrix->root;
    assert(cell);

    // while the end of the ray is not in the same cell as the goal
    // point, and we're still inside the quadtree (i.e. we have a
    // valid cell pointer)
    //while( hypot( x2-x1, y2-y1 ) >= 1.0/matrix->ppm && cell )


    while( (GTE(fabs(x2-x1),res) || GTE(fabs(y2-y1),res)) && cell )
    {
      GSList* list;
      //double keepx = x1;
      //double keepy = y1;

      /*printf( "step %d angle %.2f start (%.2f,%.2f) now (%.2f,%.2f) end (%.2f,%.2f)\n",
        step++,
        theta,
        lines[l].x1, lines[l].y1, 
        x1, y1,
        x2, y2 );
        */

      // find out where we leave the leaf node after x1,y1 and adjust
      // x1,y1 to point there

      cell = stg_find_or_create_leaf_cell(cell, x1, y1, res);
      if(cell == NULL)
      {
      //    puts("Warning, could not find a cell for a line segment... skipping the line");
          break;
      }

      // now the cell small enough, we add the object here
      cell->data = g_slist_prepend( cell->data, object );  	  
      cell->is_line = TRUE;
      cell->line_angle = theta;

      // debug rendering
      if( _render_matrix_deltas && ! cell->fig )
        stg_cell_render( cell );

      // add this object the hash table
      list = g_hash_table_lookup( matrix->ptable, object );
      list = g_slist_prepend( list, cell );
      g_hash_table_insert( matrix->ptable, object, list );      

      // now figure out where we leave this cell

      /*	  printf( "point %.2f,%.2f is in cell %p at (%.2f,%.2f) size %.2f xmin %.2f xmax %.2f ymin %.2f ymax %.2f\n",
            x1, y1, cell, cell->x, cell->y, cell->size, cell->xmin, cell->xmax, cell->ymin, cell->ymax );
            */


      //if( cell == end_cell ) // we're done 
      // break;




      if( EQ(y1,y2) ) // horizontal line
      {
        //puts( "horizontal line" );

        if( GT(x1,x2) ) // x1 gets smaller
          x1 = cell->xmin-0.001; // left edge
        else
          x1 = cell->xmax; // right edge		
      }
      else if( EQ(x1,x2) ) // vertical line
      {
        //puts( "vertical line" );

        if( GT(y1,y2) ) // y1 gets smaller
          y1 = cell->ymin-0.001; // bottom edge
        else
          y1 = cell->ymax; // max edge		
      }
      else
      {
        //print_thing( "\nbefore calc", cell, x1, y1 );

        //break;
        double c = y1 - m * x1; // line offset

        if( GT(theta,0.0) ) // up
        {
          //puts( "up" );

          // ray could leave through the top edge
          // solve x for known y      
          y1 = cell->ymax; // top edge
          x1 = (y1 - c) / m;

          // printf( "x1 %.4f = (y1 %.4f - c %.4f) / m %.4f\n", x1, y1, c, m );		  
          // if the edge crossing was not in cell bounds     
          if( !(GTE(x1,cell->xmin) && LT(x1,cell->xmax)) )
          {
            // it must have left the cell on the left or right instead 
            // solve y for known x

            if( GT(theta,M_PI/2.0) ) // left side
              x1 = cell->xmin-0.00001;
            //{ x1 = cell->xmin-0.001; puts( "left side" );}
            else // right side
              x1 = cell->xmax;
            //{ x1 = cell->xmax; puts( "right side" );}

            y1 = m * x1 + c;

            //  printf( "%.4f = %.4f * %.4f + %.4f\n",
            //      y1, m, x1, c );
          }           
          //else
          //puts( "top" );
        }	 
        else // down 
        {
          //puts( "down" );

          // ray could leave through the bottom edge
          // solve x for known y      
          y1 = cell->ymin-0.00001; // bottom edge
          x1 = (y1 - c) / m;

          // if the edge crossing was not in cell bounds     
          if( !(GTE(x1,cell->xmin) && LT(x1,cell->xmax)) )
          {
            // it must have left the cell on the left or right instead 
            // solve y for known x	  
            if( theta < -M_PI/2.0 ) // left side
              //{ x1 = cell->xmin-0.001; puts( "left side" );}
              x1 = cell->xmin-0.00001;
            else
              //{ x1 = cell->xmax; puts( "right side"); }
              x1 = cell->xmax; 

            y1 = m * x1 + c;
          }
          //else
          //puts( "bottom" );
        }

        //printf( "angle %.3f gradient %.3f entered cell at %.7f,%.7f leaving at %.7f,%.7f\n",
        //    theta, m, keepx, keepy, x1, y1 );

        // jump slightly into the next cell
        //x1 += 0.0001 * cos(theta);
        //y1 += 0.0001 * sin(theta);      

        //printf( "jumped to %.7f,%.7f\n",
        //      x1, y1 );

      }
    }
  }

  gui_unlock();

}

// render an array of [num_polys] polygons
void stg_matrix_polygons( stg_matrix_t* matrix,
    double x, double y, double a,
    stg_polygon_t* polys, int num_polys,
    void* object )
{
  int p;
  double sina, cosa;

  STG_SINCOS(a, sina, cosa);

  for( p=0; p<num_polys; ++p )
  {
    stg_polygon_t* poly =  &polys[p];

    // need at leats three points for a meaningful polygon
    int count = poly->points->len;
    if( count > 2 )
    {
      int v;
      for( v=0; v<count; ++v ) // for
      {
        stg_point_t* pt1 = &g_array_index( poly->points, stg_point_t, v );	  
        stg_point_t* pt2 = &g_array_index( poly->points, stg_point_t, (v+1) % count);

        // TODO - could be a little faster - we only really need
        // to do the geom for each point once, as the polygon is
        // a connected polyline

        stg_line_t line;

        line.x1 = x + pt1->x * cosa - pt1->y * sina;
        line.y1 = y + pt1->x * sina + pt1->y * cosa; 

        line.x2 = x + pt2->x * cosa - pt2->y * sina;
        line.y2 = y + pt2->x * sina + pt2->y * cosa; 

        stg_matrix_lines( matrix, &line, 1, object );
      }
#ifdef PRINT_NUM_POLYS_RENDERED 
      if(p > 0 && p % PRINT_NUM_POLYS_RENDERED == 0) printf("Stage: Added %lu polygons...\n", p);
#endif
    }
    else if (count == 2)
    {
      // TODO: testing. it's apparent that the ray tracer doesn't care about angle of intersection (at least for lasers and sonar), so a single line in the matrix should be detected from both sides, making a 1-line polygon a valid object
      stg_point_t* pt1 = &g_array_index( poly->points, stg_point_t, 0 );
      stg_point_t* pt2 = &g_array_index( poly->points, stg_point_t, 1 );

      stg_line_t line;

      line.x1 = x + pt1->x * cosa - pt1->y * sina;
      line.y1 = y + pt1->x * sina + pt1->y * cosa;

      line.x2 = x + pt2->x * cosa - pt2->y * sina;
      line.y2 = y + pt2->x * sina + pt2->y * cosa;

      stg_matrix_lines( matrix, &line, 1, object );
    }
    else
      PRINT_WARN( "attempted to matrix render a polygon with less than 3 points" );
  }

}

void stg_matrix_rectangle( stg_matrix_t* matrix, 
    double px, double py, double pth,
    double dx, double dy, 
    void* object )
{
  double cx, cy, sx, sy;
  double toplx, toply, toprx, topry, botlx, botly, botrx, botry;
  stg_line_t lines[4];

  dx /= 2.0;
  dy /= 2.0;

  cx = dx * cos(pth);
  cy = dy * cos(pth);
  sx = dx * sin(pth);
  sy = dy * sin(pth);

  toplx =  px + cx - sy;
  toply =  py + sx + cy;

  toprx =  px + cx + sy;
  topry =  py + sx - cy;

  botlx =  px - cx - sy;
  botly =  py - sx + cy;

  botrx =  px - cx + sy;
  botry =  py - sx - cy;

  lines[0].x1 = toplx;
  lines[0].y1 = toply;
  lines[0].x2 = toprx;
  lines[0].y2 = topry;

  lines[1].x1 = toplx;
  lines[1].y1 = toply;
  lines[1].x2 = botlx;
  lines[1].y2 = botly;

  lines[2].x1 = toprx;
  lines[2].y1 = topry;
  lines[2].x2 = botrx;
  lines[2].y2 = botry;

  lines[3].x1 = botlx;
  lines[3].y1 = botly;
  lines[3].x2 = botrx;
  lines[3].y2 = botry;

  stg_matrix_lines( matrix, lines, 4,  object );
}


stg_cell_t* stg_find_or_create_leaf_cell(stg_cell_t *cell, double x1, double y1, double leaf_cell_size)
{
  // locate the leaf cell at X,Y
  cell = stg_cell_locate(cell, x1, y1 );
  if( cell == NULL )
     return NULL;

  //stg_cell_print( cell, "" );

  // if the cell isn't small enough, we need to create children
  while( GT(cell->size, leaf_cell_size) )
  {
    const double halfcellsize = cell->size/2.0;
    const double delta = halfcellsize/2.0; //cell->size / 4.0
    const double x = cell->x;
    const double y = cell->y;
    const double x_minus_delta = x - delta;
    const double x_plus_delta = x + delta;
    const double y_minus_delta = y - delta;
    const double y_plus_delta = y + delta;
    int index;

    assert( cell->children[0] == NULL ); // make sure

    //delta = cell->size/4.0;

    cell->children[0] = stg_cell_create( cell,
        x_minus_delta,
        y_minus_delta,
        halfcellsize);
    cell->children[1] = stg_cell_create( cell,
        x_plus_delta,
        y_minus_delta,
        halfcellsize);
    cell->children[2] = stg_cell_create( cell,
        x_minus_delta,
        y_plus_delta,
        halfcellsize);
    cell->children[3] = stg_cell_create( cell,
        x_plus_delta,
        y_plus_delta,
        halfcellsize);

    // we have to drop down into a child. but which one?
    // which quadrant are we in?
    if( LT(x1,x) )
      index = LT(y1,y) ? 0 : 2; 
    else
      index = LT(y1,y) ? 1 : 3; 

    // now point to the correct child containing the point, and loop again
    cell = cell->children[ index ];           

    // debug rendering
    if( _render_matrix_deltas && ! cell->fig )
      stg_cell_render( cell );
  }

  return cell;
}

void stg_matrix_points(stg_matrix_t *matrix, 
    double x, double y, double a,
    stg_point_t *points, size_t npoints, void *object)
{
  size_t i;
  stg_cell_t *root;
  double res;
  int np = 0;
  int nc = 0;
#ifdef PRINT_NUM_POINTS_RENDERED
  puts("Stage: stg_matrix_points called..."); 
#endif
  gui_lock();
  root = matrix->root;
  res = 1.0 / matrix->ppm;
  for(i = 0; i < npoints; ++i)
  {
    GSList* list;
    stg_cell_t *cell = stg_find_or_create_leaf_cell(root, x + points[i].x, y + points[i].y, res);
    if(!cell) continue;
    // if 'object' is not already in cell's data list, add it (the same
    // point could project into the same cell pretty frequently.)
    if( g_slist_find(cell->data, object) == NULL )
    {
      cell->data = g_slist_prepend(cell->data, object);
      list = g_hash_table_lookup(matrix->ptable, object);
      list = g_slist_prepend(list, cell);
      g_hash_table_insert(matrix->ptable, object, list);
      ++nc;
    }
    cell->is_line = FALSE;
    ++np;
#ifdef PRINT_NUM_POINTS_RENDERED
    if(i > 0 && i % PRINT_NUM_POINTS_RENDERED == 0) printf("Stage: Added %lu points...\n", i);
#endif
  }
#ifdef PRINT_NUM_POINTS_RENDERED
  printf("DEBUG stg_matrix_points successfully added %d points (out of %d) to %d cells in matrix.\n", np, npoints, nc);
#endif
  gui_unlock();
}


#ifdef STG_ENABLE_MATRIX_LOCK
void stg_matrix_lock(stg_matrix_t *matrix)
{
  if(matrix->locking_enabled)
    pthread_mutex_lock(&matrix->mutex);
}

void stg_matrix_unlock(stg_matrix_t *matrix)
{
  if(matrix->locking_enabled)
    pthread_mutex_unlock(&matrix->mutex);
}

void stg_matrix_enable_lock(stg_matrix_t *matrix, gboolean enable)
{
  if(enable)
  {
    int err = pthread_mutex_init(&matrix->mutex, NULL);
    assert(err == 0);
  }
  // TODO else clear the mutex.
  matrix->locking_enabled = enable;
}
#endif

