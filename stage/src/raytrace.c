
/* To get sincos() in math.h: */
#define _GNU_SOURCE 1
#define __USE_GNU 1 


#include <stdlib.h>
#include <math.h>

#include "stage.h"
#include "stage_internal.h"

//#define DEBUG_AVERAGE_CELL_LOCATE_ITERS 1

#ifdef DEBUG_AVERAGE_CELL_LOCATE_ITERS
static long unsigned loop_count_num, loop_count_sum, loop_count_avg;
#endif

extern stg_rtk_fig_t* fig_debug_rays;

void itl_destroy( itl_t* itl )
{
  if( itl )
    {
      if( itl->incr ) 
  free( itl->incr );

      free( itl );
    }
}

itl_t* itl_reset(itl_t* itl, double x, double y, double a, double b, itl_mode_t pmode)
{
  itl->x = x;
  itl->y = y;
  itl->models = NULL;
  itl->index = 0;
  itl->range = 0;
  itl->obs_angle = 0;
  itl->incr = NULL;

  switch( pmode )
  {
    case PointToBearingRange:
    {
      // a is angle (bearing), b is range.
      itl->a = NORMALIZE(a);
      itl->max_range = b;
    }
    break;
    case PointToPoint:
    {
      // (a, b) is the other point
      double x1 = a;
      double y1 = b;           
      itl->a = atan2( y1-y, x1-x );
      itl->max_range = hypot( x1-x, y1-y );
    }
    break;
    default:
      puts( "Stage Warning: unknown LineIterator mode" );
  }
  
  STG_SINCOS(itl->a, itl->sina, itl->cosa);
  itl->tana = tan( itl->a );
  return itl;
};

itl_t* itl_create( double x, double y, double a, double b, 
       stg_matrix_t* matrix, itl_mode_t pmode )
{   
  itl_t* itl = calloc( sizeof(itl_t), 1 );
  
  itl->matrix = matrix;
  itl->x = x;
  itl->y = y;
  itl->models = NULL;
  itl->index = 0;
  itl->range = 0;  
  itl->incr = NULL;
  itl->obs_angle = 0;

  switch( pmode )
    {
    case PointToBearingRange:
      {
  double range = b;
  double bearing = a;  
  itl->a = NORMALIZE(bearing);
  itl->max_range = range;
      }
      break;
    case PointToPoint:
      {
  double x1 = a;
  double y1 = b;           
  itl->a = atan2( y1-y, x1-x );
  itl->max_range = hypot( x1-x, y1-y );
      }
      break;
    default:
      puts( "Stage Warning: unknown LineIterator mode" );
    }
  
  //printf( "a = %.2f remaining_range = %.2f\n", itl->a,
  //remaining_range ); fflush( stdout );
  
  itl->cosa = cos( itl->a );
  itl->sina = sin( itl->a );
  itl->tana = tan( itl->a );
  
  return itl;
};




// returns the first model in the array that matches, else NULL.
stg_model_t* gslist_first_matching( GSList* list, 
            stg_itl_test_func_t func, 
            stg_model_t* finder )
{
  for( ; list ; list=list->next )
    {
      if( (*func)( finder, (stg_model_t*)(list->data) ) )
  return (stg_model_t*)(list->data);
    }
  
  return NULL; // nothing in the array matched
}


void print_thing( char* prefix, stg_cell_t* cell, double x, double y )
{
  printf( "%s %p x[%.7f %.7f %.7f] y[%.7f %.7f %.7f] (x %s xmin  x %s xmax) (y %s ymin  y %s ymax)\n", 
    prefix,
    cell,     
    cell->xmin, x, cell->xmax,
    cell->ymin, y, cell->ymax,
    GTE(x,cell->xmin) ? ">=" : "<",
    LT(x,cell->xmax) ? "<" : ">=",
    GTE(y,cell->ymin) ? ">=" : "<",
    LT(y,cell->ymax) ? "<" : ">=" );
}

// in the tree that contains cell, find the smallest node at x,y. cell
// does not have to be the root (supposedly, but this seems to be buggy -rh). non-recursive for speed.
stg_cell_t* stg_cell_locate( stg_cell_t* cell, double x, double y )
{
  //printf( "\nlocate %.5f, %.5f starting from cell 0%x\n",
  //  x,y,cell );

  //print_thing( "start", cell, x, y );

  // start by going up the tree until the cell contains the point
  
#ifdef DEBUG_AVERAGE_CELL_LOCATE_ITERS 
  unsigned long loop_count = 0;
#endif

  if(!cell) return NULL;

  // if x,y is NOT contained in the cell we jump to its parent
  while( !( GTE(x,cell->xmin) && 
      LT(x,cell->xmax) && 
      GTE(y,cell->ymin) && 
      LT(y,cell->ymax) ))
    {
// XXX possible infinite loop, it's happened a few times!!
      //print_thing( "ascending", cell, x, y );
      
      if( cell->parent )
  cell = cell->parent;
      else
  return NULL; // the point is outside the root node!

#ifdef DEBUG_AVERAGE_CELL_LOCATE_ITERS 
      ++loop_count;
#endif
    }
  
  //print_thing( "after ascending", cell, x, y );

  // now we know that the point is contained in this cell, we go down
  // the tree to the leaf node that contains the point
  
if(!cell) return NULL;

  // if we have children, we must jump down into the child
  while( cell->children[0] )
    {
      //print_thing( "descending", cell, x, y );

      // choose the right quadrant 
      int index;
      if( LT(x,cell->x) )
  index = LT(y,cell->y) ? 0 : 2; 
      else
  index = LT(y,cell->y) ? 1 : 3; 

      //printf( "descent index %d {%.5f < %.5f}%d {%.5f < %.5f}%d\n",
      //      index, x, cell->x, LT(x,cell->x), y, cell->y, LT(y,cell->y));
      
      cell = cell->children[index];

#ifdef DEBUG_AVERAGE_CELL_LOCATE_ITERS 
      ++loop_count;
#endif
    }

  //print_thing( "after descending", cell, x, y );

#ifdef DEBUG_AVERAGE_CELL_LOCATE_ITERS
  loop_count_sum += loop_count;
  ++loop_count_num;
  loop_count_avg = loop_count_sum / loop_count_num;
  if(loop_count_num % 5000 == 0)
    printf("\tstg_cell_locate has done on average %2lu loop iterations\n", loop_count_avg);
#endif

  // the cell has no children and contains the point - we're done.
  return cell;
}

// Internal function that encompasses the functionality of both
// itl_first_matching and itl_all_matching. If *list is not null,
// items are appended to it until we hit the end of the ray and 
// NULL is returned. If it's null, then the first matching model 
// is returned. The list will be in *reverse* order.
stg_model_t* _itl_collect_matching( itl_t* itl, 
         stg_itl_test_func_t func, 
         stg_model_t* finder,
         GSList **list)
{
  stg_cell_t*  cell;
  double max_range;
  double x;
  double y;
  double a;
  double tana;
  gboolean a_gt_zero;
  gboolean a_gt_halfpi;
  gboolean a_lt_neg_halfpi;

  if(itl == NULL || itl->matrix == NULL || itl->matrix->root == NULL) return NULL;

  cell = itl->matrix->root;
  max_range = itl->max_range;
  x = itl->x;
  y = itl->y;
  a = itl->a;
  tana = itl->tana;
  a_gt_zero = GT(a, 0);
  a_gt_halfpi = GT(a, M_PI/2.0);
  a_lt_neg_halfpi = LT(a, -M_PI/2.0);
  itl->index = 0;
  itl->models = NULL;  
  //stg_model_t* found = NULL;


  //double xstart, ystart;

  while( LT(itl->range, max_range) )
  {
    double c, xleave, yleave;

    // locate the leaf cell at X,Y
    cell = stg_cell_locate( cell, x, y );      
      
    // the cell is null iff the point was outside the root
    if( cell == NULL )
    {
      itl->range = itl->max_range; // stop the ray here
      return NULL;
    }
      
    if( fig_debug_rays ) // draw the cell rectangle
       stg_rtk_fig_rectangle( fig_debug_rays,
             cell->x, cell->y, 0, cell->size, cell->size, 0 );
            
    if( cell->data ) 
    { 
      stg_model_t* hitmod = gslist_first_matching( (GSList*)cell->data, func, finder );
    
      if( hitmod ) 
      {
        if(cell->is_line)
        {
          itl->obs_angle = cell->line_angle;
          if(fig_debug_rays)
          {
            // write intersection angle as degrees, text with a lighter color
            char str[12];
            snprintf(str, 12, "%0.2f deg", RTOD(stg_intersection_angle(cell->line_angle, itl->a)));
            stg_rtk_fig_color_rgb32( fig_debug_rays, 0xFFBBBB );
            stg_rtk_fig_text(fig_debug_rays, cell->x+0.02, cell->y, 0, str);
            stg_rtk_fig_color_rgb32( fig_debug_rays, 0xFF0000 );
              
          }

        }

        // Found a model!!
        if(list)
          *list = g_slist_prepend(*list, (gpointer)hitmod); // prepend is O(1), append in O(N).
        else
          return hitmod; 
      }
    }
            
    c = y - tana * x; // line offset
     
    xleave = x;
    yleave = y;
      
     // if( GT(a,0) ) // up
    if(a_gt_zero)
    {
      // ray could leave through the top edge
      // solve x for known y      
      yleave = cell->ymax; // top edge
      if(tana == 0)
        xleave = 0;
      else
        xleave = (yleave - c) / tana;
    
      // if the edge crossing was not in cell bounds     
      if( !(GTE(xleave,cell->xmin) && LT(xleave,cell->xmax)) )
      {
        // it must have left the cell on the left or right instead 
        // solve y for known x
        
        //if( GT(a,M_PI/2.0) ) // left side
        if(a_gt_halfpi)
        {
          xleave = cell->xmin-0.00001;
        }
        else // right side
        {
          xleave = cell->xmax;
        }
        
        yleave = tana * xleave + c;
      }
    }   
    else 
    {
      // ray could leave through the bottom edge
      // solve x for known y      
      yleave = cell->ymin; // bottom edge
      if(tana == 0) 
        xleave = 0;
      else
        xleave = (yleave - c) / tana;
    
      // if the edge crossing was not in cell bounds     
      if( !(GTE(xleave,cell->xmin) && LT(xleave,cell->xmax)) )
      {
        // it must have left the cell on the left or right instead 
        // solve y for known x    
        //if( LT(itl->a,-M_PI/2.0) ) // left side
        if(a_lt_neg_halfpi)
        {
          xleave = cell->xmin-0.00001;
        }
        else
        {
          xleave = cell->xmax;
        }
        
        yleave = tana * xleave + c;
      }
    else
      yleave-=0.00001;
    }
      
    // jump slightly into the next cell
    //xleave += 0.0001 * itl->cosa;
    //yleave += 0.0001 * itl->sina;
      
    if( fig_debug_rays ) // draw the ray with a lighter color red
    {
      stg_rtk_fig_color_rgb32( fig_debug_rays, 0xFFBBBB );
      stg_rtk_fig_arrow_ex( fig_debug_rays, itl->x, itl->y, xleave, yleave, 0.01 );
      stg_rtk_fig_color_rgb32( fig_debug_rays, 0xFF0000 );
    }

    // jump to the leave point
    itl->range += hypot( yleave - y, xleave - x );
      
    x = itl->x = xleave;
    y = itl->y = yleave;
     
      
  }

  return NULL; // we reached the end
}


stg_model_t *itl_first_matching(itl_t *itl, stg_itl_test_func_t func, stg_model_t *finder)
{
  return _itl_collect_matching(itl, func, finder, NULL);
}

GSList *itl_all_matching(itl_t *itl, stg_itl_test_func_t func, stg_model_t *finder)
{
  GSList *list = NULL;
  _itl_collect_matching(itl, func, finder, &list);
  // reverse the list so that it goes in order from near to far
  if(list)
    list = g_slist_reverse(list);
  return list;
}

void PrintArray( GPtrArray* arr )
{
  if( arr )
  {
    int i;
    printf( "model array %p len %d\n", arr, arr->len );
    for( i=0; i<arr->len; i++ )
    printf( " (model %s)", ((stg_model_t*)g_ptr_array_index( arr, i ))->token );
  }
  else
    printf( "null array\n" );
}


